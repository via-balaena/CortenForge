//! Detect doc comments stolen from the declaration below them.
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
//! It has happened **eleven times** in this repository. Three of those were on
//! 2026-08-31, and **two of the three occurred while fixing the first** — the
//! initial fix relocated the theft onto the next function instead of removing
//! it. The eleventh was found by this gate, live in `main`, in a test file that
//! seven review rounds had read past.
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
//! Not "does every declaration have a doc" — plenty legitimately do not. The
//! signal is narrower and therefore trustworthy: **a name that HAD a doc comment
//! before the change and does NOT have one after.** That covers the motivating
//! case, a doc that silently changed owner, and the one a sweep of real history
//! turns up more of: a doc silently dropped in a refactor. Neither is a style
//! preference.
//!
//! ## Why this parses rather than scans
//!
//! An earlier version was line-based, on the argument that answering "is the
//! line above this a `///` line" survives macros and generics that would defeat
//! a naive walk. That held while only *items* were tracked. It stops holding at
//! **fields and variants**, because their declarations are textually identical
//! to expressions that are not declarations at all:
//!
//! ```text
//! pub distance: f32,      // a field DECLARATION
//! distance: 5.0,          // a struct-literal field, 25 lines away
//! distance: f32,          // a function parameter
//! Rotate,                 // an enum VARIANT
//! Rotate,                 // an argument, an array element, a `use` item
//! ```
//!
//! Telling those apart by text means tracking brace depth through string
//! literals, char literals, and block comments — **the same class of counting
//! that already carried a defect into this file twice** (see the deleted
//! `attribute_lines` and `bracket_delta`, whose miscounts made *every* item in a
//! file read as undocumented, hiding real thefts as well as inventing false
//! ones). Measured over the 703 774 tracked `.rs` lines here, a text matcher
//! over-matches badly: **29 665** lines trim to `[pub ]name: value,` against
//! 7 770 field declarations (**3.8x**), and **11 458** trim to a bare `Ident,`
//! against 1 132 variant declarations (**10.1x**). Every false hit would become
//! a trackable name.
//!
//! `syn` already answers this structurally and is already a direct dependency of
//! this crate, doing the same job for the Coverage criterion. A field
//! declaration is a `syn::Field`; a struct-literal field is a `syn::FieldValue`.
//! They are different types, so the distinction cannot be miscounted. Swept over
//! this workspace, `syn` parses **1307 of 1307** tracked `.rs` files.
//!
//! ## Coverage
//!
//! Every doc-bearing named declaration `syn` can see is tracked: items, `impl` /
//! `trait` / `extern` block members, **struct fields**, **enum variants** and
//! documented **`use` re-exports**.
//!
//! Measured by running [`docs_by_item`] itself over all 1307 tracked files, 0
//! unparsed: **13 487 documented items, 4 404 documented fields, 1 033
//! documented variants, 33 documented re-exports** — 18 957 in all. ⚠ Produced
//! by the shipped census, not by a throwaway walker written to answer the
//! question; an earlier revision of these figures came from the latter and
//! disagreed with what the code actually counts.
//!
//! ## What the change is worth, measured
//!
//! Swept over the 400 non-merge commits ending at **`793a64cd`** — 1091
//! modified-file pairs, 2182 file versions, 0 parse failures on either side —
//! against the line scanner it replaces. ⚠ The anchor is part of the claim:
//! "the last 400 commits" is a moving window that gave 1091, 1086 and 1087 on
//! three different days, none of them reproducible by a reader.
//!
//! | | findings | verified real |
//! |---|---|---|
//! | line scanner (shipped in #859) | 8 | 6 |
//! | this | **10** | **10** |
//!
//! Each of the ten was read against its diff by hand. Two are classic thefts
//! (#855, and occurrence #11 this gate found live in `main`); the other eight
//! are docs dropped during a refactor — `triage_tail` lost the four lines
//! explaining the claim its own tail line makes, `assemble_friction_wrench` lost
//! the block defining how `pose_dforce` is index-aligned. Ten findings in 400
//! commits is roughly one per 40: a rate a reviewer acts on rather than mutes.
//!
//! The four the line scanner could not see are all **fields**: #619 rewrote
//! `struct StepReport` in `concentric_lame_shells.rs` and dropped the docs off
//! four of them, including the one defining the `N_loaded` denominator.
//!
//! The two it reported wrongly are one shape, seen three times — the third being
//! a field only this engine can see at all. Splitting a big file moves a
//! documented declaration into a new module and leaves `mod <same name>;`
//! behind, so a bare-name census reads the doc as lost when it only changed
//! file. Keying on **kind plus name** removes all three, and cannot mask a
//! theft: a theft leaves the victim's own kind untouched and changes only the
//! thief above it.
//!
//! ## Known limits
//!
//! - **Macro bodies.** `syn` does not descend into a `macro_rules!` body or any
//!   other macro token stream, so a declaration written inside one is invisible.
//!   Measured at **zero occurrences** here.
//! - **`impl` blocks have no name**, so a doc on the block itself cannot be
//!   tracked. 11 carry one; their members are tracked.
//! - **Tuple-struct fields are positional**, so a doc on one has no name to
//!   track either. Measured at **zero occurrences** here.
//! - **A move across files still reads as a loss** when the name AND kind
//!   survive in the old file. Measured at **zero occurrences** across the 1091
//!   pairs above once the kind is keyed on, but it is a real shape: this check
//!   reads one file at a time and cannot see where a declaration went.
//! - **A file that does not parse is an error, not a pass.** This gate reads the
//!   working tree, so a mid-edit file will say so rather than go quiet.

use std::collections::BTreeMap;

use anyhow::Context;
use syn::visit::{self, Visit};
use syn::{Attribute, Meta};

/// How many declarations share a kind and name, and how many are documented.
///
/// ⚠ Counted, not reduced to a single bool. An earlier version kept only the
/// FIRST occurrence of a name, on the assumption that repeats were rare.
/// Measured, they are not: **102 of 1307 files** in this workspace declare the
/// same `fn` name twice (`new`, `default`, `from_str` across impl blocks), so a
/// theft on a repeated name was invisible in 8 % of the tree.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct ItemCounts {
    /// Declarations sharing this kind and name.
    pub total: usize,
    /// How many of them carry a doc comment.
    pub documented: usize,
}

/// Every named declaration in `source`, keyed by `"<kind> <name>"`, with how
/// many of each are documented.
///
/// The key is the same string the report prints, so what the census counts and
/// what a finding claims cannot drift apart.
///
/// Names are **bare**, not paths: `A::new` and `B::new` land in one bucket with
/// `total: 2`. That is deliberate. Keying on the full path would tell the two
/// apart, but it also means renaming a struct renames every field under it, so
/// every field would read as deleted-and-added and a theft inside a renamed
/// struct would be invisible. Counting occurrences catches that case without
/// introducing a blind spot during renames.
///
/// ⚠ The KIND is part of the key, and that is not cosmetic. Swept over 400
/// commits of real history it removed every false positive in 1091 changed
/// files — all three the same shape. #842 split a monolithic `lib.rs`, moving a
/// documented `pub reconstruct:` FIELD out and leaving `mod reconstruct;`
/// behind; #519 did the same to `fn contact_readout` and `fn step`. A bare name
/// reads each as a doc lost when it only changed file. A theft cannot hide
/// behind this — it leaves the victim's own kind untouched, and changes only
/// the thief above it.
///
/// # Errors
/// Returns an error if `source` is not parseable Rust. That is deliberate: an
/// unparseable file yields no declarations, and "no declarations" is
/// indistinguishable from "nothing was stolen".
pub fn docs_by_item(source: &str) -> anyhow::Result<BTreeMap<String, ItemCounts>> {
    let file = syn::parse_file(source).context("parsing Rust source")?;
    let mut census = Census::default();
    census.visit_file(&file);
    Ok(census.counts)
}

/// Accumulates the name census while `syn` walks the tree.
#[derive(Default)]
struct Census {
    counts: BTreeMap<String, ItemCounts>,
}

impl Census {
    /// Record one declaration of `kind` named `name`.
    fn record(&mut self, kind: &str, name: &syn::Ident, attrs: &[Attribute]) {
        let entry = self.counts.entry(format!("{kind} {name}")).or_default();
        entry.total += 1;
        if is_documented(attrs) {
            entry.documented += 1;
        }
    }

    /// Record a declaration whose name is optional — a tuple-struct field, or a
    /// macro invocation sitting where an item goes. Positional and anonymous
    /// declarations have nothing to key on, so they are skipped.
    fn record_opt(&mut self, kind: &str, name: Option<&syn::Ident>, attrs: &[Attribute]) {
        if let Some(name) = name {
            self.record(kind, name, attrs);
        }
    }
}

/// Whether these attributes include a doc comment.
///
/// `///` and `//!` reach `syn` as `#[doc = "..."]`, so both are caught, as is a
/// `#[doc = "..."]` written by hand — which is the same thing to `rustdoc` and
/// should be to this gate. `#[doc(hidden)]` and `#[doc(alias = "x")]` are
/// `Meta::List`, carry no prose, and are correctly excluded.
fn is_documented(attrs: &[Attribute]) -> bool {
    attrs
        .iter()
        .any(|a| a.path().is_ident("doc") && matches!(a.meta, Meta::NameValue(_)))
}

/// Visits every kind of named declaration Rust allows a doc comment on.
///
/// ⚠ Each arm must delegate to the matching `visit::visit_*` after recording,
/// or the walk stops there and everything nested below goes untracked — a
/// silent hole, since an untracked name simply never appears in the census and
/// a name that never appears can never show a loss.
///
/// The kinds NOT listed are the ones with no name to key on: `impl` blocks and
/// macro invocations in item position.
impl<'ast> Visit<'ast> for Census {
    fn visit_item_const(&mut self, i: &'ast syn::ItemConst) {
        self.record("const", &i.ident, &i.attrs);
        visit::visit_item_const(self, i);
    }

    fn visit_item_enum(&mut self, i: &'ast syn::ItemEnum) {
        self.record("enum", &i.ident, &i.attrs);
        visit::visit_item_enum(self, i);
    }

    fn visit_item_extern_crate(&mut self, i: &'ast syn::ItemExternCrate) {
        self.record("extern crate", &i.ident, &i.attrs);
        visit::visit_item_extern_crate(self, i);
    }

    fn visit_item_fn(&mut self, i: &'ast syn::ItemFn) {
        self.record("fn", &i.sig.ident, &i.attrs);
        visit::visit_item_fn(self, i);
    }

    fn visit_item_macro(&mut self, i: &'ast syn::ItemMacro) {
        // `macro_rules! name` carries an ident; `println!()` in item position
        // does not. Only the former declares anything.
        self.record_opt("macro", i.ident.as_ref(), &i.attrs);
        visit::visit_item_macro(self, i);
    }

    fn visit_item_mod(&mut self, i: &'ast syn::ItemMod) {
        self.record("mod", &i.ident, &i.attrs);
        visit::visit_item_mod(self, i);
    }

    fn visit_item_static(&mut self, i: &'ast syn::ItemStatic) {
        self.record("static", &i.ident, &i.attrs);
        visit::visit_item_static(self, i);
    }

    fn visit_item_struct(&mut self, i: &'ast syn::ItemStruct) {
        self.record("struct", &i.ident, &i.attrs);
        visit::visit_item_struct(self, i);
    }

    fn visit_item_trait(&mut self, i: &'ast syn::ItemTrait) {
        self.record("trait", &i.ident, &i.attrs);
        visit::visit_item_trait(self, i);
    }

    fn visit_item_trait_alias(&mut self, i: &'ast syn::ItemTraitAlias) {
        self.record("trait", &i.ident, &i.attrs);
        visit::visit_item_trait_alias(self, i);
    }

    fn visit_item_type(&mut self, i: &'ast syn::ItemType) {
        self.record("type", &i.ident, &i.attrs);
        visit::visit_item_type(self, i);
    }

    fn visit_item_union(&mut self, i: &'ast syn::ItemUnion) {
        self.record("union", &i.ident, &i.attrs);
        visit::visit_item_union(self, i);
    }

    /// A `use` declaration, recorded under every name it binds.
    ///
    /// ⚠ 33 `use` declarations in this workspace carry a doc comment — they are
    /// documented re-exports, and a `pub use` inserted above another below its
    /// doc robs it exactly like anything else. A glob binds no name and is
    /// skipped; a group shares one doc across its leaves, which is what the
    /// source says — so splitting a documented group into separate statements
    /// reports every leaf the doc did not follow, correctly.
    fn visit_item_use(&mut self, i: &'ast syn::ItemUse) {
        let mut names = Vec::new();
        use_names(&i.tree, &mut names);
        for name in names {
            self.record("use", name, &i.attrs);
        }
        visit::visit_item_use(self, i);
    }

    fn visit_impl_item_const(&mut self, i: &'ast syn::ImplItemConst) {
        self.record("const", &i.ident, &i.attrs);
        visit::visit_impl_item_const(self, i);
    }

    fn visit_impl_item_fn(&mut self, i: &'ast syn::ImplItemFn) {
        self.record("fn", &i.sig.ident, &i.attrs);
        visit::visit_impl_item_fn(self, i);
    }

    fn visit_impl_item_type(&mut self, i: &'ast syn::ImplItemType) {
        self.record("type", &i.ident, &i.attrs);
        visit::visit_impl_item_type(self, i);
    }

    fn visit_trait_item_const(&mut self, i: &'ast syn::TraitItemConst) {
        self.record("const", &i.ident, &i.attrs);
        visit::visit_trait_item_const(self, i);
    }

    fn visit_trait_item_fn(&mut self, i: &'ast syn::TraitItemFn) {
        self.record("fn", &i.sig.ident, &i.attrs);
        visit::visit_trait_item_fn(self, i);
    }

    fn visit_trait_item_type(&mut self, i: &'ast syn::TraitItemType) {
        self.record("type", &i.ident, &i.attrs);
        visit::visit_trait_item_type(self, i);
    }

    fn visit_foreign_item_fn(&mut self, i: &'ast syn::ForeignItemFn) {
        self.record("fn", &i.sig.ident, &i.attrs);
        visit::visit_foreign_item_fn(self, i);
    }

    fn visit_foreign_item_static(&mut self, i: &'ast syn::ForeignItemStatic) {
        self.record("static", &i.ident, &i.attrs);
        visit::visit_foreign_item_static(self, i);
    }

    fn visit_foreign_item_type(&mut self, i: &'ast syn::ForeignItemType) {
        self.record("type", &i.ident, &i.attrs);
        visit::visit_foreign_item_type(self, i);
    }

    /// ★ A struct, union or variant field DECLARATION.
    ///
    /// ⚠ This is the whole reason the check parses instead of scanning. `syn`
    /// uses `Field` only in a declaration; a struct literal's `distance: 5.0`
    /// is a `FieldValue` and a function's `distance: f32` is a `PatType`.
    /// Neither reaches here, so neither can be mistaken for a field.
    fn visit_field(&mut self, i: &'ast syn::Field) {
        self.record_opt("field", i.ident.as_ref(), &i.attrs);
        visit::visit_field(self, i);
    }

    /// ★ An enum variant declaration — never a match arm, which is an `Arm`.
    fn visit_variant(&mut self, i: &'ast syn::Variant) {
        self.record("variant", &i.ident, &i.attrs);
        visit::visit_variant(self, i);
    }
}

/// Every name a `use` tree binds, ignoring globs.
///
/// A rename binds its alias, not its source: `use a::B as C` introduces `C`.
fn use_names<'a>(tree: &'a syn::UseTree, out: &mut Vec<&'a syn::Ident>) {
    match tree {
        syn::UseTree::Path(p) => use_names(&p.tree, out),
        syn::UseTree::Name(n) => out.push(&n.ident),
        syn::UseTree::Rename(r) => out.push(&r.rename),
        syn::UseTree::Glob(_) => {}
        syn::UseTree::Group(g) => {
            for t in &g.items {
                use_names(t, out);
            }
        }
    }
}

/// Declarations that had a doc comment in `before` and have none in `after`,
/// each as the `"<kind> <name>"` the report prints.
///
/// Declarations added or removed by the change are ignored — only a name
/// present on both sides can have *lost* anything.
///
/// # Errors
/// Returns an error if either side fails to parse.
pub fn items_that_lost_docs(before: &str, after: &str) -> anyhow::Result<Vec<String>> {
    let b = docs_by_item(before)?;
    let a = docs_by_item(after)?;
    Ok(b.into_iter()
        .filter(|(name, before_counts)| {
            let Some(after_counts) = a.get(name) else {
                // Every declaration of that name is gone — a deletion, not a theft.
                return false;
            };
            // Fewer documented than before, while at least as many declarations
            // exist: something that had a doc no longer does, and it was not
            // removed.
            //
            // ⚠ The `total` guard is what separates theft from deletion. Without
            // it, deleting one documented `new` out of two would report as a
            // loss — a false positive in 8 % of this workspace's files.
            after_counts.documented < before_counts.documented
                && after_counts.total >= before_counts.total
        })
        .map(|(name, _)| name)
        .collect())
}

/// Run `git -C repo <args>`, failing on a non-zero exit.
///
/// ⚠⚠ FAILS CLOSED. `.output()?` propagates only a SPAWN failure; a non-zero
/// git exit still returns `Ok` with empty stdout, and empty means "no changed
/// files" or "no before-text" — both of which report clean. Same shape as the
/// `.unwrap_or_default()` this replaced at the call sites, and as the `|| true`
/// removed from `xtask/hooks/pre-commit`. A guard that goes quiet when its
/// inputs break is worse than no guard, because it also stops anyone looking.
fn git_in(repo: &std::path::Path, args: &[&str]) -> anyhow::Result<String> {
    let out = std::process::Command::new("git")
        .arg("-C")
        .arg(repo)
        .args(args)
        .output()
        .with_context(|| format!("running git {}", args.join(" ")))?;
    if !out.status.success() {
        anyhow::bail!(
            "git {} failed ({}): {}",
            args.join(" "),
            out.status,
            String::from_utf8_lossy(&out.stderr).trim()
        );
    }
    Ok(String::from_utf8_lossy(&out.stdout).into_owned())
}

/// What one sweep looked at, and what it found.
///
/// ★ The counts are returned rather than only printed. A run that skipped every
/// file and reported nothing looks identical to a clean run otherwise — the "an
/// empty result is not evidence" failure this whole check exists to answer — so
/// the accounting has to be assertable, not just visible. Each is counted where
/// it happens; none is derived from the others.
#[derive(Debug, Default, PartialEq, Eq)]
struct Sweep {
    /// `(file, "<kind> <name>")` for each declaration that lost its doc.
    findings: Vec<(String, String)>,
    /// Changed `.rs` files the diff listed.
    files: usize,
    /// Files actually compared before-against-after.
    compared: usize,
    /// Files with no base version — added by this change.
    skipped_added: usize,
    /// Files with no working-tree version — deleted by this change.
    skipped_deleted: usize,
}

/// Compare every `.rs` file changed between `base` and `HEAD` in `repo`.
///
/// ⚠ Needs real history: `base...HEAD` requires the merge-base, so a shallow
/// checkout cannot run this. In CI it lives in a job fetching `fetch-depth: 0`
/// for exactly that reason.
///
/// # Errors
/// Returns an error if git cannot be run, if there is no merge-base, or if a
/// changed file does not parse.
fn sweep(repo: &std::path::Path, base: &str) -> anyhow::Result<Sweep> {
    use std::process::Command;

    // ⚠ `git diff --name-only` yields REPO-RELATIVE paths, but the working-tree
    // reads below resolve against `repo`. Anchor on the repo root instead: run
    // from a subdirectory and every path would miss, each file would count as
    // "deleted", and the check would report clean with a theft in front of it.
    let root = git_in(repo, &["rev-parse", "--show-toplevel"])?
        .trim()
        .to_string();

    // ⚠ NOT through the strict helper: `git merge-base` exits NON-ZERO when the
    // two commits share no ancestor, and prints nothing to stderr for it.
    // Routing it through `git_in` bailed with a bare "failed (exit status: 1):"
    // and buried the one message that tells a CI operator what to change. Empty
    // output is still fatal below, so this tolerates the status without
    // becoming fail-open.
    let merge_base = {
        let out = Command::new("git")
            .arg("-C")
            .arg(repo)
            .args(["merge-base", base, "HEAD"])
            .output()
            .context("running git merge-base")?;
        String::from_utf8_lossy(&out.stdout).trim().to_string()
    };
    if merge_base.is_empty() {
        anyhow::bail!(
            "no merge-base between {base} and HEAD — this needs full history \
             (fetch-depth: 0), not a shallow checkout"
        );
    }

    // ⚠ No `HEAD` here, deliberately: `git diff --name-only <base>` includes
    // UNCOMMITTED work, and the content below is read from the working tree.
    // Listing committed changes while reading the working tree would disagree
    // locally — a theft you had not yet committed would be invisible to the very
    // check meant to catch it before you commit.
    let changed = git_in(repo, &["diff", "--name-only", &merge_base])?;
    let files: Vec<&str> = changed.lines().filter(|f| f.ends_with(".rs")).collect();

    // The two legitimate "nothing to compare" cases are distinguished
    // explicitly, by asking git and the filesystem rather than by inferring
    // emptiness: a file ADDED by this change has no base version, and one
    // DELETED by it has no working-tree version. Anything else is an error.
    let mut out = Sweep {
        files: files.len(),
        ..Sweep::default()
    };

    for file in &files {
        let blob = format!("{merge_base}:{file}");

        // ⚠ stderr silenced: a MISS here is the expected "file is new to this
        // change" path, and git prints `fatal: path ... exists on disk, but not
        // in <sha>` for it. A `fatal:` in a CI log reads as breakage, and a
        // check that cries wolf on its own happy path is one people learn to
        // scroll past. Only the exit status is consulted.
        let in_base = Command::new("git")
            .arg("-C")
            .arg(repo)
            .args(["cat-file", "-e", &blob])
            .stderr(std::process::Stdio::null())
            .status()
            .with_context(|| format!("git cat-file -e {blob}"))?
            .success();
        if !in_base {
            out.skipped_added += 1;
            continue;
        }

        let path = std::path::Path::new(&root).join(file);
        if !path.exists() {
            out.skipped_deleted += 1;
            continue;
        }

        let before = git_in(repo, &["show", &blob])?;
        let after = std::fs::read_to_string(&path)
            .with_context(|| format!("reading {file} from the working tree"))?;

        // ⚠ The file name belongs in the error. A bare `expected one of ...`
        // from `syn` with no path is unactionable in a CI log listing 40 files.
        let lost = items_that_lost_docs(&before, &after)
            .with_context(|| format!("checking {file} for stolen doc comments"))?;
        out.compared += 1;
        for item in lost {
            out.findings.push(((*file).to_string(), item));
        }
    }

    // ⚠ COUNTED where the comparison happens, not derived as
    // `files - skipped_added - skipped_deleted`. Subtraction makes the total
    // add up by construction, so a file that was neither compared nor skipped —
    // a `continue` added to the loop later — would still report a tidy,
    // self-consistent number. That is the failure this line exists to prevent,
    // so it must be measured rather than inferred. The identity is asserted
    // instead, where a break in it is loud.
    //
    // ⚠ `assert_eq!`, NOT `debug_assert_eq!`. CI invokes this as
    // `cargo run --release`, where a debug assertion is compiled out — the one
    // configuration that matters would have had no check at all, while the
    // comment above claimed it did. It costs three integer adds once per run,
    // not per file.
    assert_eq!(
        out.compared + out.skipped_added + out.skipped_deleted,
        out.files,
        "every changed file must be compared or explicitly skipped"
    );
    Ok(out)
}

/// Run the check over `repo` and report it.
///
/// Takes the directory rather than reading the current one, so the whole check
/// is reachable from a test. ⚠ It used to resolve the CWD itself, which left
/// every line below unreachable except by running the binary: a mutation sweep
/// replaced this entire function with `Ok(())` and the suite stayed green.
///
/// # Errors
/// Returns an error if the sweep fails, or if any declaration lost a doc.
pub fn run(repo: &std::path::Path, base: &str) -> anyhow::Result<()> {
    let s = sweep(repo, base)?;

    println!(
        "→ Checked {} changed Rust file(s) for stolen doc comments…",
        s.files
    );
    // ★ Say what was NOT compared.
    println!(
        "  compared {} file(s); skipped {} added, {} deleted",
        s.compared, s.skipped_added, s.skipped_deleted
    );

    if s.findings.is_empty() {
        println!("✓ No doc comments changed owner");
        return Ok(());
    }

    eprintln!("\n✗ Doc comment(s) stolen from the declaration below them:\n");
    for (file, item) in &s.findings {
        eprintln!("    {file}: `{item}` LOST the doc comment it had before this change");
    }
    eprintln!(
        "\n  A declaration inserted above another, but below its doc comment, takes\n  \
         that comment with it. The only insertion point in a Rust item list that\n  \
         cannot steal a doc is between a `use` block and the next doc comment.\n"
    );
    anyhow::bail!("{} doc comment(s) changed owner", s.findings.len())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The findings for a change, or a panic naming the parse error.
    fn lost(before: &str, after: &str) -> Vec<String> {
        items_that_lost_docs(before, after).expect("both sides must parse")
    }

    /// One expected finding.
    fn one(kind_and_name: &str) -> Vec<String> {
        vec![kind_and_name.to_string()]
    }

    #[test]
    fn an_item_that_keeps_its_doc_is_not_reported() {
        let src = "/// Doc.\nfn a() {}\n";
        assert!(lost(src, src).is_empty());
    }

    #[test]
    fn an_item_that_never_had_a_doc_is_not_reported() {
        // The check is about LOSS, not about coverage. Plenty of declarations
        // are legitimately undocumented and must not create noise.
        assert!(lost("fn a() {}\n", "fn a() {}\nfn b() {}\n").is_empty());
    }

    #[test]
    fn attributes_between_a_doc_and_its_item_do_not_hide_the_doc() {
        // ⚠ Without this, every `#[test]` would read as undocumented and real
        // losses would drown in false positives. The line-based scanner needed
        // a bracket-counting walk-up to get here; `syn` attaches attributes and
        // docs to the same item by construction.
        let src = "/// Doc.\n#[test]\n#[allow(clippy::all)]\nfn a() {}\n";
        assert_eq!(docs_by_item(src).unwrap()["fn a"].documented, 1);
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
            lost(before, after),
            one("fn build_funnel_solid_apex_axial_is_none_now_integral"),
            "the gate must catch the instance that motivated it"
        );
    }

    #[test]
    fn the_real_2026_09_01_theft_in_main_is_detected() {
        // ★★★ Occurrence #11, found by this gate live in `main` and fixed in
        // #860: a `const` inserted below `critical_issue_types`' doc took it.
        // Kept as a regression case because it is the first theft NO review
        // round found — seven of them read past it.
        let before = "\
/// The issue types a CSG robustness probe treats as critical.
fn critical_issue_types() -> Vec<u8> { vec![] }
";
        let after = "\
/// The issue types a CSG robustness probe treats as critical.
const CSG_ROBUSTNESS_CRITICALS: usize = 3;

fn critical_issue_types() -> Vec<u8> { vec![] }
";
        assert_eq!(lost(before, after), one("fn critical_issue_types"));
    }

    #[test]
    fn the_real_2026_09_01_field_doc_loss_is_detected() {
        // ★★★ Found by sweeping 400 commits with fields tracked, in a file the
        // shipped gate had already passed: #619 rewrote `struct StepReport` and
        // dropped the docs off four of its fields, including the one defining
        // the `N_loaded` denominator. Invisible to every version before this one.
        let before = "\
struct StepReport {
    /// Mean radial displacement over every cavity-surface vertex.
    cavity_u_r_mean: f64,
    /// Newton iteration count at convergence.
    iter_count: usize,
}
";
        let after = "\
struct StepReport {
    cavity_u_r_mean: f64,
    iter_count: usize,
}
";
        assert_eq!(
            lost(before, after),
            vec![
                "field cavity_u_r_mean".to_string(),
                "field iter_count".to_string()
            ]
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
        assert!(lost(before, after).is_empty());
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
        assert_eq!(lost(before, after), one("fn b"));
    }

    #[test]
    fn every_doc_bearing_kind_is_tracked() {
        // ★ Exhaustiveness probe. Each row is one declaration WITH a doc, the
        // same declaration WITHOUT it, and the key it must be reported under. A
        // kind that is not visited yields no entry, reports nothing, and fails
        // here by name.
        //
        // ⚠ It is written as doc REMOVAL rather than item insertion on purpose.
        // The old version used `const fn a()` to cover `const`, which exercises
        // the QUALIFIER and not the item kind — every documented constant in the
        // workspace was invisible while a test claimed otherwise.
        for (documented, stripped, key) in [
            ("/// D.\nfn a() {}", "fn a() {}", "fn a"),
            ("/// D.\nstruct a;", "struct a;", "struct a"),
            ("/// D.\nenum a {}", "enum a {}", "enum a"),
            ("/// D.\ntrait a {}", "trait a {}", "trait a"),
            ("/// D.\ntype a = u8;", "type a = u8;", "type a"),
            ("/// D.\nunion a { x: u8 }", "union a { x: u8 }", "union a"),
            (
                "/// D.\npub async unsafe fn a() {}",
                "pub async unsafe fn a() {}",
                "fn a",
            ),
            (
                "/// D.\npub(crate) const fn a() -> u8 { 0 }",
                "pub(crate) const fn a() -> u8 { 0 }",
                "fn a",
            ),
            ("/// D.\nconst a: u8 = 1;", "const a: u8 = 1;", "const a"),
            (
                "/// D.\nstatic mut a: u8 = 1;",
                "static mut a: u8 = 1;",
                "static a",
            ),
            ("/// D.\nmod a {}", "mod a {}", "mod a"),
            ("/// D.\nmacro_rules! a {}", "macro_rules! a {}", "macro a"),
            (
                "/// D.\nextern crate a;",
                "extern crate a;",
                "extern crate a",
            ),
            ("/// D.\npub use m::a;", "pub use m::a;", "use a"),
            ("/// D.\npub use m::z as a;", "pub use m::z as a;", "use a"),
            // ⚠ These two were absent, and a mutation sweep proved it: replacing
            // `visit_item_trait_alias` or `visit_foreign_item_type` with `()`
            // survived the whole suite. The table tested `trait a {}` (an
            // `ItemTrait`) and `extern "C" { fn }` / `{ static }`, never the
            // alias or the foreign type — an exhaustiveness probe that was not.
            ("/// D.\ntrait a = b;", "trait a = b;", "trait a"),
            (
                "extern \"C\" { /// D.\n type a; }",
                "extern \"C\" { type a; }",
                "type a",
            ),
            // ★ The two kinds this change exists for.
            (
                "struct S { /// D.\n a: u8 }",
                "struct S { a: u8 }",
                "field a",
            ),
            ("enum E { /// D.\n a }", "enum E { a }", "variant a"),
            (
                "enum E { V { /// D.\n a: u8 } }",
                "enum E { V { a: u8 } }",
                "field a",
            ),
            // Members of `impl`, `trait` and `extern` blocks.
            (
                "impl S { /// D.\n fn a() {} }",
                "impl S { fn a() {} }",
                "fn a",
            ),
            (
                "impl S { /// D.\n const a: u8 = 1; }",
                "impl S { const a: u8 = 1; }",
                "const a",
            ),
            (
                "impl S { /// D.\n type a = u8; }",
                "impl S { type a = u8; }",
                "type a",
            ),
            (
                "trait T { /// D.\n fn a(); }",
                "trait T { fn a(); }",
                "fn a",
            ),
            (
                "trait T { /// D.\n const a: u8; }",
                "trait T { const a: u8; }",
                "const a",
            ),
            (
                "trait T { /// D.\n type a; }",
                "trait T { type a; }",
                "type a",
            ),
            (
                "extern \"C\" { /// D.\n fn a(); }",
                "extern \"C\" { fn a(); }",
                "fn a",
            ),
            (
                "extern \"C\" { /// D.\n static a: u8; }",
                "extern \"C\" { static a: u8; }",
                "static a",
            ),
            // Nested inside a function body: proves each visitor delegates to
            // `visit::visit_*` instead of stopping the walk where it recorded.
            (
                "fn outer() { /// D.\n struct a; }",
                "fn outer() { struct a; }",
                "struct a",
            ),
        ] {
            assert_eq!(
                lost(documented, stripped),
                one(key),
                "kind not tracked: {documented}"
            );
        }
    }

    #[test]
    fn a_theft_from_a_struct_field_is_caught() {
        // ★★★ The case this change exists for. Demonstrated against the real
        // tree before the fix: inserting `zoom_bias` above `distance` in
        // `cf-bevy-common/src/camera.rs` stole its doc and the shipped gate said
        // "✓ No doc comments changed owner".
        let before = "\
pub struct Orbit {
    /// Metres from the focus.
    pub distance: f32,
}
";
        let after = "\
pub struct Orbit {
    /// Metres from the focus.
    pub zoom_bias: f32,

    pub distance: f32,
}
";
        assert_eq!(lost(before, after), one("field distance"));
    }

    #[test]
    fn a_theft_from_an_enum_variant_is_caught() {
        let before = "\
pub enum Step {
    /// Import a scan.
    AddScan,
}
";
        let after = "\
pub enum Step {
    /// Import a scan.
    Calibrate,

    AddScan,
}
";
        assert_eq!(lost(before, after), one("variant AddScan"));
    }

    #[test]
    fn a_theft_from_a_documented_re_export_is_caught() {
        // ⚠ The third gap the same sweep found: 33 `use` declarations here carry
        // a doc comment, and a `pub use` inserted below one robs it exactly like
        // an item does.
        let before = "/// Re-exported for callers.\npub use inner::Mold;\n";
        let after = "/// Re-exported for callers.\npub use inner::Cast;\n\npub use inner::Mold;\n";
        assert_eq!(lost(before, after), one("use Mold"));
    }

    #[test]
    fn a_struct_literal_field_is_not_a_declaration() {
        // ⚠⚠ The trap that made a text scan unusable here. `zoom: 1.0` is
        // textually a field declaration and is not one. Measured: 29 665 lines
        // in this workspace trim to `[pub ]name: value,` against 7 770 field
        // declarations — a 3.8x over-match, every hit of which would become a
        // trackable name.
        let src = "\
struct Config {
    zoom: f32,
}

fn make() -> Config {
    Config {
        zoom: 1.0,
    }
}
";
        assert_eq!(
            docs_by_item(src).unwrap()["field zoom"].total,
            1,
            "the literal's `zoom` must not be counted as a second declaration"
        );
    }

    #[test]
    fn a_function_parameter_is_not_a_field() {
        let src = "fn set(\n    distance: f32,\n) {\n}\n";
        assert!(
            !docs_by_item(src).unwrap().contains_key("field distance"),
            "a parameter is a `PatType`, not a `Field`"
        );
    }

    #[test]
    fn a_match_arm_is_not_a_variant() {
        // ⚠ 11 458 lines in this workspace trim to a bare `Ident,` against
        // 1 132 variant declarations — a 10.1x over-match: call arguments,
        // array elements, `use` lists and shorthand struct literals share the
        // shape.
        let src = "\
enum Mode {
    Rotate,
}

fn f(m: Mode) {
    match m {
        Mode::Rotate => {}
    }
}
";
        assert_eq!(docs_by_item(src).unwrap()["variant Rotate"].total, 1);
    }

    #[test]
    fn a_declaration_that_changes_kind_is_not_reported() {
        // ⚠⚠ The only false positive in the whole 400-commit sweep, and why the
        // kind is part of the key. #842 split a monolithic `lib.rs`: the
        // documented `pub reconstruct:` FIELD moved to another file, and
        // `mod reconstruct;` stayed behind under the same name. On bare names
        // that reads as "reconstruct lost its doc"; it did not, it changed file.
        // field -> mod, from #842's split of `cf-scan-prep-core`.
        let before = "\
struct PrepToml {
    /// Provenance for floor reconstruction.
    pub reconstruct: Option<u8>,
}
";
        assert!(
            lost(before, "mod reconstruct;\n").is_empty(),
            "a move is not a theft"
        );

        // fn -> mod, from #519's split of the `StaggeredCoupling` impl. Two of
        // the three real occurrences took this form.
        let before = "impl C {\n    /// Contact readout.\n    fn contact_readout(&self) {}\n}\n";
        assert!(
            lost(before, "mod contact_readout;\n").is_empty(),
            "a move is not a theft"
        );
    }

    #[test]
    fn adding_a_field_is_not_a_theft() {
        // The correct form of the change above: the new field goes below the
        // existing one's doc-and-declaration pair and steals nothing.
        let before = "struct S {\n    /// Kept.\n    a: u8,\n}\n";
        let after = "struct S {\n    /// Kept.\n    a: u8,\n\n    /// New.\n    b: u8,\n}\n";
        assert!(lost(before, after).is_empty());
    }

    #[test]
    fn deleting_a_documented_field_is_not_a_theft() {
        // ⚠ The `total` guard, on the new surface: removing a field removes its
        // doc too, and that is not a loss of ownership.
        let before = "struct S {\n    /// Gone.\n    a: u8,\n    /// Kept.\n    b: u8,\n}\n";
        let after = "struct S {\n    /// Kept.\n    b: u8,\n}\n";
        assert!(lost(before, after).is_empty(), "a deletion is not a theft");
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
            lost(before, after),
            one("fn new"),
            "the SECOND `new` lost its doc; only counting occurrences can see that"
        );
    }

    #[test]
    fn deleting_one_of_two_documented_namesakes_is_not_reported() {
        // ⚠ The false positive the `total` guard prevents. Without it, removing
        // one documented `new` of two reads as a loss — noise in 8 % of files,
        // which is how a gate gets switched off.
        let before = "/// One.\nfn new() {}\n\n/// Two.\nfn new() {}\n";
        assert!(lost(before, "/// One.\nfn new() {}\n").is_empty());
    }

    #[test]
    fn a_deleted_item_is_not_a_loss() {
        // Deleting a documented item removes its doc too; that is not theft.
        assert!(lost("/// Doc.\nfn gone() {}\n", "").is_empty());
    }

    #[test]
    fn a_multi_line_attribute_does_not_hide_the_doc_above_it() {
        // ⚠ The R5 false positive. The line scanner walked up with
        // `starts_with("#[")`, which stops at the CLOSING `)]` line, so the item
        // read as undocumented and merely ADDING a multi-line derive reported as
        // theft. 101 such attributes across 77 files here, two already under a
        // doc. Structurally impossible now: the attribute and the doc hang off
        // the same item.
        let before = "/// Doc.\npub struct Widget;\n";
        let after = "/// Doc.\n#[derive(\n    Debug,\n)]\npub struct Widget;\n";
        assert!(lost(before, after).is_empty());
    }

    #[test]
    fn a_theft_under_a_multi_line_attribute_is_still_caught() {
        // The negative control for the case above: tolerating attributes must
        // not blind the check.
        let before = "/// Doc.\n#[derive(\n    Debug,\n)]\npub struct Widget;\n";
        let after =
            "/// Doc.\n#[derive(\n    Debug,\n)]\npub struct Thief;\n\npub struct Widget;\n";
        assert_eq!(lost(before, after), one("struct Widget"));
    }

    #[test]
    fn braces_and_brackets_in_strings_cannot_poison_the_census() {
        // ⚠ The R6 runaway, and the reason fields are not matched by text.
        // `#[doc = "unmatched [ here"]` left the old bracket depth stuck above
        // zero, so every later line read as an attribute, the walk-up climbed to
        // the top of the file, and EVERY item read as undocumented — which hides
        // real thefts as well as inventing false ones, since a file already at
        // zero documented can never show a decrease. 14 460 lines here carry
        // format-string braces and 103 441 are unbalanced on their own line.
        let src = "\
#[doc = \"unmatched [ here\"]
pub struct A;

fn noisy() {
    let _ = format!(\"{{ unbalanced {x}\", x = 1);
    // } a closing brace in a comment
}

/// Kept.
pub struct B;
";
        let m = docs_by_item(src).unwrap();
        assert_eq!(
            m["struct B"].documented, 1,
            "B's own doc must survive the noise"
        );
        assert_eq!(
            m["struct A"].documented, 1,
            "`#[doc = \"...\"]` IS a doc comment — the old scanner said otherwise"
        );
    }

    #[test]
    fn doc_hidden_is_not_a_doc_comment() {
        // The negative control for the assertion above: `#[doc(hidden)]` and
        // `#[doc(alias = ...)]` carry no prose and must not count, or removing
        // one would report as a theft.
        let src = "#[doc(hidden)]\n#[doc(alias = \"x\")]\npub struct A;\n";
        assert_eq!(docs_by_item(src).unwrap()["struct A"].documented, 0);
    }

    #[test]
    fn a_blank_line_does_not_detach_a_doc() {
        // ⚠ Verified against `#![deny(missing_docs)]`: a blank line between a
        // doc and its item does NOT detach it. The line scanner needed an
        // explicit rule for this and for two other separators, and shipped
        // knowing only one of the three.
        let before = "/// Doc.\npub const ITEM: u8 = 1;\n";
        let after = "/// Doc.\n\npub const ITEM: u8 = 1;\n";
        assert!(lost(before, after).is_empty());
    }

    #[test]
    fn a_plain_comment_does_not_detach_a_doc() {
        // ⚠ The R8 false positive, found by sweeping 149 real files. Rust strips
        // `//` comments before doc attributes bind, so this item is documented.
        // Treating one as a separator left **305 items in this workspace**
        // reading as undocumented — a false positive whenever such a note is
        // added, and a false NEGATIVE for all 305.
        let before = "/// Doc.\nconst INSET: f64 = 1.0;\n";
        let after = "/// Doc.\n// an explanatory note added later\nconst INSET: f64 = 1.0;\n";
        assert!(lost(before, after).is_empty());
    }

    #[test]
    fn a_theft_across_a_separator_is_still_caught() {
        // Negative control for both cases above: tolerating blank lines and `//`
        // notes must not blind the check.
        let before = "/// Doc.\n\n// note\npub const ITEM: u8 = 1;\n";
        let after = "/// Doc.\n\n// note\npub const THIEF: u8 = 9;\n\npub const ITEM: u8 = 1;\n";
        assert_eq!(lost(before, after), one("const ITEM"));
    }

    #[test]
    fn a_documented_module_is_tracked() {
        // ⚠ The R8 gap. `mod` was absent from the line scanner's kind list, so
        // every documented module could lose a doc undetected. The census
        // counts 84 of them; #859's note said 75, which was the line scanner's
        // own count and missed the inline and nested ones it could not see.
        let before = "/// Module doc.\npub mod prelude {}\n";
        let after = "/// Module doc.\npub mod thief {}\n\npub mod prelude {}\n";
        assert_eq!(lost(before, after), one("mod prelude"));
    }

    #[test]
    fn an_unparseable_side_is_an_error_not_a_pass() {
        // ⚠⚠ FAILS CLOSED. An unparseable file yields no declarations, and "no
        // declarations" is indistinguishable from "nothing was stolen" — the
        // "an empty result is not evidence" shape this whole check exists for.
        assert!(
            items_that_lost_docs("/// Doc.\nfn a() {}\n", "fn a( {\n").is_err(),
            "a file that does not parse must not report clean"
        );
        assert!(items_that_lost_docs("fn a( {\n", "fn a() {}\n").is_err());
    }

    // ── The sweep itself, driven end to end over a real git repo ──────────
    //
    // ⚠⚠ Everything below exists because a mutation sweep replaced the WHOLE of
    // `run` with `Ok(())` and the suite stayed green: 12 of 14 survivors sat in
    // this one function. The pure core was well covered and the part that
    // actually runs in CI had no test at all — a gate whose decision lives only
    // behind a CI job, which is the shape that rots silently.

    /// Run a git command in `dir`, asserting it succeeded.
    fn g(dir: &std::path::Path, args: &[&str]) {
        let out = std::process::Command::new("git")
            .arg("-C")
            .arg(dir)
            .args(args)
            .output()
            .expect("git is on PATH");
        assert!(
            out.status.success(),
            "git {args:?} failed: {}",
            String::from_utf8_lossy(&out.stderr)
        );
    }

    /// A throwaway repo under the system temp dir — never the real one.
    fn temp_repo(name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("cf-doc-theft-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        g(&dir, &["init", "-q", "-b", "main"]);
        g(&dir, &["config", "user.email", "gate@example.invalid"]);
        g(&dir, &["config", "user.name", "gate"]);
        dir
    }

    fn write(dir: &std::path::Path, name: &str, body: &str) {
        std::fs::write(dir.join(name), body).expect("write");
    }

    fn commit(dir: &std::path::Path, msg: &str) {
        g(dir, &["add", "-A"]);
        g(dir, &["commit", "-q", "--no-verify", "-m", msg]);
    }

    #[test]
    fn sweep_catches_a_theft_in_a_real_repo() {
        let d = temp_repo("theft");
        write(&d, "a.rs", "/// Victim doc.\nfn victim() {}\n");
        commit(&d, "base");
        // Uncommitted — the shape the pre-commit hook has to catch.
        write(
            &d,
            "a.rs",
            "/// Victim doc.\nfn thief() {}\n\nfn victim() {}\n",
        );

        let s = sweep(&d, "HEAD").expect("sweep runs");
        assert_eq!(s.files, 1);
        assert_eq!(s.compared, 1);
        assert_eq!(s.skipped_added, 0);
        assert_eq!(s.skipped_deleted, 0);
        assert_eq!(
            s.findings,
            vec![("a.rs".to_string(), "fn victim".to_string())]
        );
        assert!(run(&d, "HEAD").is_err(), "run must FAIL on a theft");
    }

    #[test]
    fn sweep_is_clean_when_nothing_changed_owner() {
        let d = temp_repo("clean");
        write(&d, "a.rs", "/// Victim doc.\nfn victim() {}\n");
        commit(&d, "base");
        write(
            &d,
            "a.rs",
            "/// New doc.\nfn newcomer() {}\n\n/// Victim doc.\nfn victim() {}\n",
        );

        let s = sweep(&d, "HEAD").expect("sweep runs");
        assert!(s.findings.is_empty(), "the correct form steals nothing");
        assert_eq!(s.compared, 1, "and it must actually have been compared");
        assert!(run(&d, "HEAD").is_ok());
    }

    #[test]
    fn the_accounting_separates_compared_from_added_and_deleted() {
        // ★ The counters and the `compared` arithmetic. Mutating `+=` to `*=`
        // or `-=`, and `-` to `+` or `/`, all survived — including on the line
        // whose only job is to say what was NOT looked at.
        let d = temp_repo("accounting");
        write(&d, "a.rs", "/// Victim doc.\nfn victim() {}\n");
        write(&d, "gone.rs", "fn gone() {}\n");
        commit(&d, "base");
        g(&d, &["branch", "base"]);

        write(&d, "new.rs", "fn fresh() {}\n");
        commit(&d, "add a file");

        write(
            &d,
            "a.rs",
            "/// Victim doc.\nfn thief() {}\n\nfn victim() {}\n",
        );
        std::fs::remove_file(d.join("gone.rs")).expect("remove");

        let s = sweep(&d, "base").expect("sweep runs");
        assert_eq!(s.files, 3, "a.rs modified, gone.rs deleted, new.rs added");
        assert_eq!(s.skipped_added, 1, "new.rs has no base version");
        assert_eq!(s.skipped_deleted, 1, "gone.rs has no working-tree version");
        assert_eq!(s.compared, 1, "only a.rs could be compared");
        assert_eq!(s.findings.len(), 1);
    }

    #[test]
    fn a_git_failure_is_an_error_not_a_clean_sweep() {
        // ⚠⚠ The fail-closed guarantee the module header advertises in bold,
        // and which nothing tested. Deleting the `!` in `git_in` makes every
        // git failure return `Ok("")`, and empty reads as "nothing to compare".
        let dir = std::env::temp_dir().join(format!("cf-doc-theft-norepo-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");

        let err = sweep(&dir, "HEAD").expect_err("outside a repo this must fail");
        let msg = format!("{err:#}");
        assert!(
            msg.contains("rev-parse"),
            "the git failure must surface as ITSELF, not as a later symptom: {msg}"
        );
    }

    #[test]
    fn a_missing_merge_base_names_the_shallow_checkout() {
        // The one git call deliberately NOT routed through the strict helper,
        // so this message stays reachable instead of becoming a bare
        // "failed (exit status: 1):".
        let d = temp_repo("nomergebase");
        write(&d, "a.rs", "fn a() {}\n");
        commit(&d, "base");

        let err = sweep(&d, "no-such-ref").expect_err("an unknown base must fail");
        let msg = format!("{err:#}");
        assert!(msg.contains("merge-base"), "got: {msg}");
        assert!(msg.contains("fetch-depth"), "must name the fix: {msg}");
    }
}
