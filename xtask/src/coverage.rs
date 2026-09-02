//! Production-only line coverage: the denominator the Coverage criterion is about.
//!
//! The instrumented run measures *test* binaries (see [`crate::coverage_run`]
//! for how), so a crate's own `#[cfg(test)]` code lands in the report alongside
//! the production code it exercises. Counting it makes the criterion measure
//! two unrelated things:
//!
//! - test bodies that **run** are very nearly 100 % covered, so they inflate
//!   the numerator — measured across seven crates, this was worth up to
//!   +12 percentage points;
//! - test bodies that **cannot** run — `#[ignore]`d licence-gated gates, and
//!   the helpers reachable only from them — sit in the denominator
//!   contributing zero, so improving a crate's instrumentation *lowers* its
//!   letter.
//!
//! Both distortions have one cause and one fix: measure only production lines,
//! which is what "≥75 % coverage" was always meant to assert.
//!
//! Three pieces do that. [`is_own_production_file`] decides which files in the
//! export are this crate's production sources at all; [`cfg_test_spans`] finds
//! the line ranges within them that exist only under `cfg(test)`; and
//! [`mapped_lines`] reproduces llvm-cov's own line-coverage rule so those ranges
//! can be subtracted line by line.
//!
//! [`declares_no_production_code`] sits slightly outside that story. It answers
//! a question the criterion also needs — not "which lines count" but "was there
//! anything here to measure at all" — and it reads the crate's source rather
//! than an export, because the crates it exists for never produce one. Its own
//! rationale explains why that case is otherwise indistinguishable from a
//! measurement that broke.

use std::collections::{BTreeMap, HashSet};
use std::path::{Path, PathBuf};

use syn::punctuated::Punctuated;
use syn::spanned::Spanned;
use syn::{Attribute, Fields, ImplItem, Item, Meta, Token, TraitItem};

/// Production line coverage for one file of a crate.
///
/// The crate-level percentage says a crate needs tests; this says *where* to
/// write them. Both come out of the same instrumented run — the per-file split
/// is already in the export and was simply being summed away, so surfacing it
/// costs no extra measurement. That matters: a coverage run is minutes per
/// crate, so a breakdown gated behind a second run would rarely be taken.
#[derive(Debug)]
pub(crate) struct FileCoverage {
    /// Path relative to the crate root (`src/solver/pgs.rs`), not the absolute
    /// path llvm-cov emits. The crate is already named by the report around it.
    pub file: String,
    /// Production lines in this file executed at least once.
    pub covered: u64,
    /// Production lines in this file instrumented. Always > 0 — a file that
    /// contributes no production lines is not a triage target and is omitted.
    pub total: u64,
    /// This file could not be read or parsed, so its `#[cfg(test)]` lines were
    /// never subtracted and are counted here as production.
    ///
    /// The counts above are therefore an over-estimate of both the work and the
    /// gap, by an unknown amount — `#[ignore]`d gates read as uncovered
    /// production code. The crate-level report already says *how many* files
    /// this happened to; the flag says *which*, because the ranking is what a
    /// reader acts on and an inflated row can outrank every honest one.
    pub test_lines_counted: bool,
    /// This file belongs to a BINARY target rather than the library — see
    /// [`is_bin_target_file`].
    ///
    /// Recorded, not acted on: these lines stay in the graded total exactly as
    /// before. It exists so the split can be *reported* while the decision to
    /// exclude binary lines from a library bar is still open — a number nobody
    /// can see is not evidence for that decision.
    pub is_bin: bool,
}

impl FileCoverage {
    /// Production lines never executed — the triage ranking key.
    pub fn uncovered(&self) -> u64 {
        self.total.saturating_sub(self.covered)
    }

    /// Percentage covered.
    ///
    /// Returns a bare `f64` rather than an `Option` like
    /// [`ProductionCoverage::percent`], because [`production_coverage`] never
    /// emits a row with a zero denominator — a file contributing no production
    /// lines is dropped instead. That is an invariant of the producer, not of
    /// this struct, whose fields are constructible directly; a hand-built row
    /// with `total: 0` yields `NaN` here rather than a panic.
    pub fn percent(&self) -> f64 {
        100.0 * self.covered as f64 / self.total as f64
    }
}

/// Production line coverage for one crate, plus what it could not account for.
#[derive(Debug, Default)]
pub(crate) struct ProductionCoverage {
    /// Production lines executed at least once.
    pub covered: u64,
    /// Production lines instrumented.
    pub total: u64,
    /// Test lines removed from both sides — reported, never silent.
    ///
    /// Three sources: `#[cfg(test)]` spans, files pulled in by a
    /// `#[cfg(test)] mod name;`, and this crate's `tests/` integration source,
    /// which reaches the export because those targets are instrumented.
    pub excluded: u64,
    /// Files whose source could not be read or parsed, so their test code was *not*
    /// excluded. Their lines are still counted, which understates coverage —
    /// the safe direction, but the grade must say so rather than imply a
    /// clean sweep.
    pub unparsed: Vec<String>,
    /// The same measurement split by file, **most uncovered lines first**, so a
    /// reader takes the biggest win first. Ties break on path so two runs over
    /// one tree print in the same order.
    ///
    /// Sums to [`Self::covered`] / [`Self::total`] by construction: both are
    /// accumulated in the same pass over the same lines.
    pub files: Vec<FileCoverage>,
    /// The subset of [`Self::covered`] contributed by BINARY targets.
    pub bin_covered: u64,
    /// The subset of [`Self::total`] contributed by BINARY targets.
    ///
    /// ⚠ A **subset**, not a separate pool: binary lines are counted in
    /// `covered`/`total` as they always were, and recorded here as well. So
    /// [`Self::percent`] is bit-identical to what it returned before this
    /// field existed, and no crate's grade moves because the split was added.
    pub bin_total: u64,
}

impl ProductionCoverage {
    /// Percentage, or `None` when the crate has no production lines at all.
    ///
    /// **The graded figure, and deliberately unchanged.** It spans library and
    /// binary lines alike; [`Self::lib_percent`] is the alternative, reported
    /// beside it but not yet gating.
    pub fn percent(&self) -> Option<f64> {
        (self.total > 0).then(|| 100.0 * self.covered as f64 / self.total as f64)
    }

    /// Percentage over LIBRARY lines only — binary-target lines removed from
    /// both sides. `None` when the crate has no library lines.
    ///
    /// Reported, never graded (yet). The case for it: `.run()`, `Cli::parse()`
    /// and a `main` that ends in an event loop cannot be unit-tested at all, so
    /// a crate whose binary is a large share of its source is held to a bar its
    /// library could clear and its binary structurally cannot. Measured on
    /// `cf-viewer` 2026-08-16 — 460 of 1082 production lines are `src/main.rs`
    /// at 1.5 % — the whole-crate figure is 33.8 % and the library-only figure
    /// is 57.7 %.
    ///
    /// ⚠ The case against, which is why this only reports: excluding binary
    /// lines makes `main.rs` a place where logic stops being measured. That is
    /// the dead-zone shape #772–#774 closed, so the exclusion cannot land as a
    /// silent default — it needs the split visible in the triage table first,
    /// which is what this PR does.
    pub fn lib_percent(&self) -> Option<f64> {
        let total = self.total.saturating_sub(self.bin_total);
        let covered = self.covered.saturating_sub(self.bin_covered);
        (total > 0).then(|| 100.0 * covered as f64 / total as f64)
    }

    /// The smallest `covered` that would meet `percent` — the bar, in LINES.
    ///
    /// ⚠⚠ Integer, and that is the point. [`Self::margin_lines`] is a line
    /// count and [`Self::meets`] is the comparison the LETTER comes from, so
    /// the two have to name the same boundary line-for-line. Deriving the
    /// letter from an `f64` percentage and the margin from integers lets them
    /// disagree at exactly the boundary — which is the only place either
    /// number is interesting, and where the crates this reporting exists for
    /// live. [`Self::percent`] survives as display only.
    ///
    /// `div_ceil`, not `/`: the bar is the smallest integer `covered`
    /// satisfying `covered * 100 >= percent * total`, and truncating division
    /// would put the bar one line low whenever the quotient is fractional —
    /// reporting a crate as exactly at the bar while [`Self::meets`] says it
    /// is under. That disagreement is the whole defect this is shaped to
    /// avoid.
    pub fn bar(&self, percent: u64) -> u64 {
        (percent * self.total).div_ceil(100)
    }

    /// Does the measurement meet `percent`? Decided in integers.
    ///
    /// `covered * 100 >= percent * total` is `covered >= bar(percent)` — the
    /// same question, and provably the same answer, which is what lets the
    /// letter and the margin be read together.
    ///
    /// A crate with no production lines answers `true` (`0 >= 0`). No caller
    /// reaches that: `coverage_result` returns on `percent()` being `None`
    /// first, and it is `None` on exactly the same condition. Documented
    /// rather than guarded, because the guard belongs at the one place that
    /// has somewhere better to go — a crate with nothing to measure is not a
    /// crate that met the bar, and it should not be graded as though it were.
    pub fn meets(&self, percent: u64) -> bool {
        self.covered * 100 >= percent * self.total
    }

    /// Covered lines above the bar for `percent`; NEGATIVE means short by that
    /// many. `None` when the crate has no production lines, matching
    /// [`Self::percent`].
    ///
    /// ★ Lines, not percentage points, because lines are the unit the reader
    /// acts in ("write four more tests"), the unit the run-to-run spread is
    /// measured in ([`Self::noise_floor_lines`]), and an integer — so there is
    /// no rounding ambiguity at the bar, which is where every use of this
    /// number is.
    pub fn margin_lines(&self, percent: u64) -> Option<i64> {
        (self.total > 0).then(|| self.covered as i64 - self.bar(percent) as i64)
    }

    /// How many covered lines two runs over the SAME TREE can differ by.
    ///
    /// A margin inside this spread is not evidence the crate is above the bar:
    /// re-running could lose it without a line of source changing.
    ///
    /// **Measured, at both ends:**
    ///
    /// - `cf-codesign`, 2026-08-16: 1416 → 1410 covered of 1622 — 6 lines,
    ///   0.37 %.
    /// - `sim-soft`, 2026-08-29: five runs on identical trees spanned
    ///   8198–8216 covered of 9146 — 18 lines, 0.20 %.
    ///
    /// So the spread scales with the crate rather than sitting at a constant.
    /// A flat 6 lines is a third of sim-soft's observed drift; a flat 0.4 % is
    /// four lines on an 800-line crate (3.2, rounded up), under the smallest
    /// drift anyone has actually seen. Hence **0.4 %, or 6 lines, whichever is larger** — the
    /// proportional term sizes the big crates, the floor sizes the small ones,
    /// and neither measurement is contradicted.
    ///
    /// Suspected mechanism, UNVERIFIED: solver tests iterate to convergence,
    /// so thread scheduling changes which lines execute — drift in the TESTS,
    /// not in `llvm-cov`. Named as a suspicion, not a finding; the band is
    /// sized from the observed spread either way, so nothing here depends on
    /// the cause being right.
    ///
    /// ⚠ Both observations are from ONE machine. A CI runner with a different
    /// core count may well spread wider, which would make this floor too
    /// tight. It errs toward flagging fewer crates than it should — the
    /// direction that stays quiet, not the direction that lies.
    pub fn noise_floor_lines(&self) -> u64 {
        /// Smallest drift ever directly observed (`cf-codesign`, 6 of 1622).
        const OBSERVED_FLOOR: u64 = 6;
        /// 0.4 %, as parts per thousand so the arithmetic stays integer.
        const PER_MILLE: u64 = 4;
        OBSERVED_FLOOR.max((self.total * PER_MILLE).div_ceil(1000))
    }
}

/// Strip the crate root from an absolute export path: `src/solver/pgs.rs`.
///
/// `rfind`, not `find`: only the LAST occurrence is the crate root. A checkout
/// living under a directory that repeats the crate path — `/home/sim/L0/core/…/
/// sim/L0/core/src/foo.rs` — would otherwise keep most of the absolute path.
///
/// Falls back to the full name when the marker is absent. Callers filter with
/// [`is_own_production_file`] first, so that cannot happen today; returning the
/// path unshortened is the harmless answer if it ever does.
fn relative_to_crate(name: &str, crate_path: &str) -> String {
    match name.rfind(crate_path) {
        Some(at) => name[at + crate_path.len()..]
            .trim_start_matches('/')
            .to_string(),
        None => name.to_string(),
    }
}

/// Whether an export filename is one of the crate's own PRODUCTION sources.
///
/// Under `crate_path`, but NOT under its `tests/` directory. Integration test
/// source is compiled as its own crate and is instrumented only so its binary
/// emits a profile ([`crate::coverage_run`]) — it is not the code under
/// measurement, and counting it would pad both sides of the ratio with
/// ~100 %-covered test bodies, the same distortion this module strips
/// `#[cfg(test)]` to avoid.
///
/// Lives here rather than beside the instrumentation because "what counts as
/// production" is this module's question; `coverage_run` only needs the answer.
pub(crate) fn is_own_production_file(name: &str, crate_path: &str) -> bool {
    // ⚠ Trim BEFORE composing the `tests/` path, not after. `path_is_under`
    // trims only the END of what it is handed, so a `crate_path` arriving with
    // a trailing slash would compose to `mesh/mesh//tests` — an interior `//`
    // that matches nothing, silently switching the exclusion off and counting
    // the crate's OWN integration tests as production. That is the same
    // fail-open this function exists to prevent, reintroduced one level up.
    // `find_crate_path` does not currently produce a trailing slash, so this
    // is a guard rather than a live fix — but `path_is_under` accepts one by
    // design, which makes it a caller's reasonable assumption.
    let crate_path = crate_path.trim_end_matches('/');
    path_is_under(name, crate_path) && !path_is_under(name, &format!("{crate_path}/tests"))
}

/// Whether `name` names a file inside the directory `dir`.
///
/// ★ **Boundary-aware on purpose — do NOT "simplify" this back to
/// `name.contains(dir)`.** That is what it used to be, and a bare `contains`
/// cannot tell a crate from a sibling whose path merely extends it: with
/// `dir = "mesh/mesh"` it matches `mesh/mesh-io/src/stl.rs`. That is not a
/// corner case here — 38 pairs of workspace members collide this way, and
/// `mesh/mesh` alone swallows 16 of them, every `mesh-*` crate included.
///
/// Two ways it went wrong, both measured on 74882bf6:
///
/// - COVERAGE, and this is the fail-OPEN one: a sibling's files landed in the
///   umbrella crate's ratio. Worse, a sibling's `tests/` came in as PRODUCTION,
///   because the exclusion above strips only `<crate_path>/tests/` and
///   `mesh/mesh-io/tests/x.rs` does not contain `mesh/mesh/tests/`. Near-100 %
///   covered test bodies then padded the numerator of a crate that never ran
///   them.
/// - CLIPPY (`grade::any_span_in_crate`): one unused variable injected into
///   `mesh-io` produced three diagnostics, and all three were attributed to
///   `mesh`, whose own source was clean and untouched.
///
/// Matches a relative path by prefix and an absolute one by embedded segment,
/// so callers may pass either — [`crate::grade::find_crate_path`] normally
/// returns workspace-relative but falls back to absolute.
pub(crate) fn path_is_under(name: &str, dir: &str) -> bool {
    let name = name.replace('\\', "/");
    let dir = dir.replace('\\', "/");
    let dir = dir.trim_end_matches('/');
    if dir.is_empty() {
        // `""` or `"/"` — the filesystem root, which every path is under. This
        // is a true answer rather than a swallowed edge case, and it preserves
        // what the old `name.contains("")` did. ⚠ It is also match-everything,
        // so it is worth knowing it is unreachable from grading: the only
        // producer is `find_crate_path`, which strips `workspace_root` off a
        // member's manifest dir, and the root manifest here is VIRTUAL — no
        // member sits at the root, so no member strips to the empty string.
        return true;
    }
    let seg = format!("{dir}/");
    name.starts_with(&seg) || name.contains(&format!("/{seg}"))
}

/// Whether a crate-relative path is the root of a BINARY target, by Cargo's
/// own auto-discovery convention: `src/main.rs`, `src/bin/<name>.rs`, and
/// `src/bin/<name>/main.rs`.
///
/// Takes the path already made relative by [`relative_to_crate`], so a
/// workspace checked out under a directory called `src` cannot match.
///
/// ⚠ **Deliberately under-inclusive, in the safe direction.** Two shapes it
/// does not catch:
///
/// - A `[[bin]] path = …` pointing outside the convention. Re-measured
///   2026-09-02 across all 301 members: **9** explicit `[[bin]]` declarations
///   in 9 crates, and every one of them sets `path = "src/main.rs"` — so no
///   declaration in this workspace exercises the miss. (The `mesh-io/fuzz`
///   crate does declare four off-convention `fuzz_targets/*.rs` bins, but
///   cargo-fuzz gives it its own `[workspace]` table, so it is not a member
///   here and never reaches grading.)
/// - A module reached by `mod foo;` **from** a binary root. `src/foo.rs` is
///   indistinguishable from a library module by path, and resolving it would
///   mean parsing every binary's module tree.
///
/// ⚠ It also inherits [`is_own_production_file`]'s bare-substring crate match,
/// and this workspace has live collisions: `tools/cf-studio` is a substring of
/// `tools/cf-studio-gui`, `-core` and `-engine`. Measured 2026-08-17, it does
/// not fire — grading `cf-studio` returns exactly two rows, its own `src/lib.rs`
/// and `src/main.rs`, summing to its reported 174 — because instrumentation is
/// scoped to the measured crate, so a sibling's source never reaches the
/// export for the filter to mis-admit. The filter is a second line of defence
/// that real input does not currently exercise.
///
/// ★ And were it to fire, it fails safe HERE specifically:
/// [`relative_to_crate`] would strip the shared prefix and leave
/// `-gui/src/main.rs`, which matches none of the arms below, so a sibling's
/// binary would be counted as library code — understating the library figure
/// rather than inflating it.
///
/// A third miss, on Windows: the separator is assumed to be `/`, so a
/// backslashed export path matches nothing here. Inherited rather than introduced —
/// [`relative_to_crate`] searches for a `/`-shaped `crate_path` and would fail
/// to strip such a name in the first place, so the whole per-file split shares
/// the assumption.
///
/// ★ **Measured 2026-08-17, and none of the three occurs in this workspace.**
/// Of 301 members, 239 have a `src/main.rs` and in every one of them cargo
/// really does build it as a binary target — so this never fires on a library
/// file. And **zero** bin targets across the workspace sit off the convention,
/// so nothing is currently missed either. The hedges below are about staying
/// safe if that changes, not about a gap that exists.
///
/// All of these misses classify binary code as library code, which counts it
/// against the library bar. That direction can only ever make a crate look
/// worse, so no gap here can produce a false pass — the property worth having,
/// given the exclusion this feeds is precisely the kind that creates a dead
/// zone.
pub(crate) fn is_bin_target_file(relative: &str) -> bool {
    relative == "src/main.rs"
        || (relative.starts_with("src/bin/")
            && (relative.ends_with("/main.rs") || relative.matches('/').count() == 2))
}

/// Whether `crate_path`'s own sources are **positively established** to declare
/// no production code at all.
///
/// ★ The question this answers is "was there anything here to instrument?", and
/// it exists to tell two very different failures apart. When no test binary
/// writes a `.profraw`, [`crate::coverage_run`] reports that instrumentation
/// never reached the crate — the defect #770 was written to catch, where the
/// binaries linked an instrumented library but were not instrumented themselves.
/// A crate with no production code produces the identical symptom for the
/// opposite reason: there are no functions, so nothing carries a coverage map.
/// Graded the same way, the empty crate reads as an `F` for bad coverage.
///
/// The shape that has it today is the bench companion crate —
/// `sim-core-benches`, `mesh-repair-benches`, `mesh-shell-benches`,
/// `sim-ml-chassis-benches` — whose `src/lib.rs` is a doc comment and nothing
/// else. All their content is `benches/*.rs` under `harness = false`, which
/// coverage does not measure and is not meant to.
///
/// ★ **Measured, not inferred, and it is not about scoping.** Built with
/// *blanket* `RUSTFLAGS=-C instrument-coverage` — not this crate's scoped
/// wrapper — `sim-core-benches`' unit-test binary carries **zero**
/// `__llvm_covmap` / `__llvm_prf_*` sections, and running it with
/// `LLVM_PROFILE_FILE` set writes **no** `.profraw`. So the missing profile is
/// intrinsic to a crate with no functions; instrumenting harder cannot produce
/// one, and no change to [`crate::coverage_run`]'s scoping would.
///
/// ★★ **Cross-checked against the whole workspace, by an instrument that shares
/// no code with this one.** Run over all 47 coverage-graded crates, this
/// predicate names exactly seven: the four bench crates above, plus the facades
/// `cortenforge`, `mesh` and `sim`. The 2026-08-16 sizing run — which compiles,
/// instruments, runs the binaries and reads llvm-cov's export — produced no
/// percentage for exactly those same seven and a percentage for all forty
/// others. The sets are equal, so on real input there is no crate this calls
/// empty that the measurement finds code in.
///
/// ★ The agreement is unforced, which is what makes it worth recording: the
/// three facades reach their N/A through [`production_coverage`]'s export path
/// and never reach this function at all, so nothing was fitted to them.
///
/// ⚠ **Deliberately conservative: it answers `false` whenever it cannot prove
/// the negative.** A missing or empty `src/`, a file it cannot read or parse,
/// and any error raised while walking the tree all yield `false`, and the caller
/// keeps reporting the failure. Only a clean walk that finds nothing can excuse
/// a crate, so no real instrumentation defect can be relabelled as an empty
/// crate by a parse error or an unreadable directory.
///
/// ⚠ One known conservative miss, harmless by construction: a crate whose only
/// content is a non-inline `#[cfg(test)] mod tests;` reads as production code,
/// because the gate is on the declaration in the parent while the walk judges
/// `tests.rs` on its own items. [`test_only_files`] resolves exactly that
/// pattern for the line accounting; wiring it in here would buy nothing, since
/// the answer only ever errs toward keeping the failure report.
pub(crate) fn declares_no_production_code(crate_path: &Path) -> bool {
    let src = crate_path.join("src");
    if !src.is_dir() {
        // "(no src/)" is a different verdict, reached elsewhere. Claiming
        // anything from here would overlap two criteria on one crate.
        return false;
    }

    let mut saw_a_source_file = false;
    // `follow_links` so a symlinked module is read rather than skipped as a
    // non-file, and every walk error is fatal rather than filtered away. Both
    // are the same point: a subtree this cannot read is a subtree that might
    // hold the crate's whole implementation, and dropping the error would let
    // "I could not look" return as "there was nothing there". A link cycle
    // surfaces here as an error too, which is the answer we want.
    for entry in walkdir::WalkDir::new(&src).follow_links(true) {
        let Ok(entry) = entry else {
            return false;
        };
        let path = entry.path();
        if !entry.file_type().is_file() || path.extension() != Some("rs".as_ref()) {
            continue;
        }
        saw_a_source_file = true;
        let Ok(text) = std::fs::read_to_string(path) else {
            return false;
        };
        let Ok(file) = syn::parse_file(&text) else {
            return false;
        };
        if declares_code(&file.items) {
            return false;
        }
    }

    // An empty `src/` is not a crate that compiles; treat it as unproven.
    saw_a_source_file
}

/// True when any of `items` would compile to something a coverage map can name.
///
/// `use` and `extern crate` emit no code. A non-inline `mod name;` emits none
/// *here* — its file is walked in its own right, so recursing would double-count
/// and, worse, a missing file would read as emptiness.
///
/// ⚠ Only `mod` and `fn` are checked for `#[cfg(test)]`; every other item kind
/// counts as production whatever gates it. That is the conservative direction —
/// it can only keep a crate out of the empty-crate verdict — and it covers the
/// idiom that actually occurs, a file whose sole item is `#[cfg(test)] mod
/// tests`. Enumerating the attributes of all nineteen `syn::Item` variants to
/// close the rest would be a lot of surface for a case this workspace does not
/// contain.
fn declares_code(items: &[Item]) -> bool {
    items.iter().any(|item| match item {
        Item::Use(_) | Item::ExternCrate(_) => false,
        Item::Fn(f) => !is_cfg_test(&f.attrs),
        Item::Mod(m) if is_cfg_test(&m.attrs) => false,
        Item::Mod(m) => match &m.content {
            Some((_, inner)) => declares_code(inner),
            None => false,
        },
        _ => true,
    })
}

/// True when this `cfg` predicate can hold *only* in a test build.
///
/// `all(...)` needs every conjunct, so one `test` conjunct settles it.
/// `any(...)` and `not(...)` do not: the item still exists in some non-test
/// build, so it stays in the denominator as production code.
///
/// This parses the predicate rather than matching on its text. A substring
/// test reads `all(feature = "test-fixtures", ...)` — a feature this workspace
/// actually has — as a test gate, and silently drops shipped code from the
/// metric, which looks like a measured crate and is not.
fn requires_test(meta: &Meta) -> bool {
    match meta {
        Meta::Path(p) => p.is_ident("test"),
        Meta::List(l) if l.path.is_ident("all") => l
            .parse_args_with(Punctuated::<Meta, Token![,]>::parse_terminated)
            .is_ok_and(|inner| inner.iter().any(requires_test)),
        _ => false,
    }
}

/// True if these attributes gate the item on `test`.
fn is_cfg_test(attrs: &[Attribute]) -> bool {
    attrs.iter().any(|a| {
        a.path().is_ident("cfg") && a.parse_args::<Meta>().is_ok_and(|m| requires_test(&m))
    })
}

fn push(out: &mut Vec<(usize, usize)>, sp: proc_macro2::Span) {
    out.push((sp.start().line, sp.end().line));
}

fn walk_fields(fields: &Fields, out: &mut Vec<(usize, usize)>) {
    for f in fields.iter() {
        if is_cfg_test(&f.attrs) {
            push(out, f.span());
        }
    }
}

fn walk(items: &[Item], out: &mut Vec<(usize, usize)>) {
    for it in items {
        let attrs: &[Attribute] = match it {
            Item::Fn(f) => &f.attrs,
            Item::Mod(m) => &m.attrs,
            Item::Struct(s) => &s.attrs,
            Item::Enum(e) => &e.attrs,
            Item::Impl(i) => &i.attrs,
            Item::Const(c) => &c.attrs,
            Item::Static(s) => &s.attrs,
            Item::Type(t) => &t.attrs,
            Item::Trait(t) => &t.attrs,
            Item::Use(u) => &u.attrs,
            _ => &[],
        };
        if is_cfg_test(attrs) {
            push(out, it.span());
            continue; // the whole subtree is test-only
        }
        match it {
            Item::Mod(m) => {
                if let Some((_, inner)) = &m.content {
                    walk(inner, out);
                }
            }
            Item::Impl(i) => {
                for ii in &i.items {
                    let a: &[Attribute] = match ii {
                        ImplItem::Fn(f) => &f.attrs,
                        ImplItem::Const(c) => &c.attrs,
                        ImplItem::Type(t) => &t.attrs,
                        _ => &[],
                    };
                    if is_cfg_test(a) {
                        push(out, ii.span());
                    }
                }
            }
            Item::Trait(t) => {
                for ti in &t.items {
                    if let TraitItem::Fn(f) = ti {
                        if is_cfg_test(&f.attrs) {
                            push(out, ti.span());
                        }
                    }
                }
            }
            Item::Struct(s) => walk_fields(&s.fields, out),
            Item::Enum(e) => {
                for v in &e.variants {
                    if is_cfg_test(&v.attrs) {
                        push(out, v.span());
                    } else {
                        walk_fields(&v.fields, out);
                    }
                }
            }
            _ => {}
        }
    }
}

/// Inclusive `(first, last)` line ranges that exist only under `cfg(test)`.
///
/// Returns `None` if the source does not parse.
///
/// A range starts at the item's first token, which is its first attribute —
/// doc comments included. That only widens the range upward across attribute
/// and doc lines, which carry no executable code of their own; it would take
/// two items sharing a source line for this to reach a neighbour's code.
/// Checked rather than assumed: across the seven measured crates, **no**
/// production function has a single excluded line.
///
/// ⚠ **Known limit: `#[cfg(test)]` in expression position is not seen.** This
/// walks items, so a test-gated *field initialiser* inside a production
/// function — `sim/L1/fsu-model/src/lib.rs:663` — stays in the denominator.
/// Bounded by counting: of the 333 files under `sim/`, `design/` and `mesh/`
/// that use `#[cfg(test)]`, that site is the only one, so the whole class is
/// worth **one line workspace-wide**. Walking every function body to reclaim
/// it would cost far more than it is worth.
pub(crate) fn cfg_test_spans(src: &str) -> Option<Vec<(usize, usize)>> {
    let file = syn::parse_file(src).ok()?;
    let mut out = Vec::new();
    walk(&file.items, &mut out);
    Some(out)
}

/// Where `mod name;` inside `owner` puts the child's source.
///
/// Both spellings are returned; the caller keeps whichever exists.
fn child_module_candidates(owner: &Path, name: &str) -> Vec<PathBuf> {
    let Some(dir) = owner.parent() else {
        return Vec::new();
    };
    // `mod.rs`, `lib.rs` and `main.rs` own their own directory; any other file
    // owns a subdirectory named after itself.
    let base = match owner.file_stem().and_then(|s| s.to_str()) {
        Some("mod" | "lib" | "main") => dir.to_path_buf(),
        Some(stem) => dir.join(stem),
        None => return Vec::new(),
    };
    vec![
        base.join(format!("{name}.rs")),
        base.join(name).join("mod.rs"),
    ]
}

/// Names declared by a non-inline `mod name;`, split by whether the
/// declaration is `#[cfg(test)]`.
fn declared_modules(src: &str) -> Option<(Vec<String>, Vec<String>)> {
    let file = syn::parse_file(src).ok()?;
    let (mut test, mut plain) = (Vec::new(), Vec::new());
    fn visit(items: &[Item], test: &mut Vec<String>, plain: &mut Vec<String>) {
        for it in items {
            let Item::Mod(m) = it else { continue };
            match &m.content {
                // Inline modules keep their code in this file, where the span
                // walk already sees it.
                Some((_, inner)) => visit(inner, test, plain),
                None => {
                    let name = m.ident.to_string();
                    if is_cfg_test(&m.attrs) {
                        test.push(name);
                    } else {
                        plain.push(name);
                    }
                }
            }
        }
    }
    visit(&file.items, &mut test, &mut plain);
    Some((test, plain))
}

/// The crate's directory on disk, recovered from one of the export's absolute
/// filenames.
///
/// llvm-cov reports absolute paths that contain `crate_path` as a fragment;
/// cutting one at the end of that fragment leaves the crate root, which is where
/// [`test_only_files`] starts its walk.
///
/// ★ Anchored at the LAST occurrence, matching [`relative_to_crate`]. A checkout
/// under a directory that repeats the crate's own path fragment would otherwise
/// resolve to an ANCESTOR of the crate, whose `src/lib.rs` is some other crate's
/// or nothing at all. Either way this crate's module tree goes unwalked and the
/// seeding falls back to the export's file list — the very gap it exists to
/// close. Seeding a foreign tree cannot mismark anything, since the result is
/// only ever consulted for files already established as this crate's own.
fn crate_root_of(name: &str, crate_path: &str) -> Option<PathBuf> {
    name.rfind(crate_path)
        .map(|at| PathBuf::from(&name[..at + crate_path.len()]))
}

/// Every source file that exists only for tests because some ancestor declared
/// it with `#[cfg(test)] mod name;`.
///
/// Nothing *inside* such a file is attributed to `cfg(test)` — the gate is on
/// the declaration, in a different file — so the span walk alone would score
/// the whole file as production code.
///
/// ★ **The search starts from the crate's module tree, not from the export's
/// file list, and that distinction is the whole guard.** A declaration-only
/// `mod.rs` — nothing but `pub mod` and `pub use` — compiles to no regions, so
/// llvm-cov omits it from `files[]`. Seeding only from listed files therefore
/// never opens it, never sees the `#[cfg(test)] mod name;` it holds, and scores
/// every line of the test file that declaration gates as production code.
/// Following ordinary `mod name;` declarations down from the crate root is what
/// reaches such a file.
///
/// ⚠ **The omitted-declaration case is real and it is not small.** `sim-gpu`
/// declares all three of its test modules in `src/pipeline/mod.rs`: 53 lines of
/// module doc, `pub mod`, `pub use` and the three declarations themselves, with
/// no executable code — so it carries no regions, and llvm-cov did not list it
/// among the 18 files it reported for the crate. The 2026-08-18 weekly
/// sweep (run 32106341473) scored the crate 344/5647 lines with "10 test lines
/// excluded", and ranked `src/pipeline/tests.rs` (1511 uncovered),
/// `conformance_tests.rs` (491) and `contact_conformance_tests.rs` (406) as its
/// three largest production gaps. All three are test files.
///
/// ⛔ An earlier version of this doc argued the guard could not move the number,
/// because llvm-cov omits `#[cfg(test)] mod` files from `files[]` — observed in
/// `sim-mjcf`, whose `src/parser/tests.rs` is absent from the export while still
/// attributing 93 executed functions to it. That run falsifies the argument: the
/// omission is per-file and undocumented, and for `sim-gpu` all three files were
/// listed. The guard is load-bearing.
fn test_only_files(crate_root: Option<&Path>, listed: &[String]) -> HashSet<PathBuf> {
    let mut found = HashSet::new();
    let mut test_queue: Vec<PathBuf> = Vec::new();

    // Production side of the walk. `lib.rs`/`main.rs` are the roots Cargo
    // compiles from; the listed files are added because a binary-only crate has
    // no `src/lib.rs` to descend from, and its root carries `fn main` and so is
    // always in the export. A root that could not be recovered contributes
    // nothing rather than a cwd-relative guess.
    //
    // ⚠ `walked` and the `is_file()` guard below are both correctness-neutral
    // here — `found` is written only by the test walk, and `read_to_string`
    // rejects a non-file anyway. They bound the work instead: `walked` keeps a
    // deep tree from being re-parsed once per path that reaches it. The test
    // walk's `is_file()` is NOT equivalent, because it gates `found.insert`.
    let mut prod_queue: Vec<PathBuf> = crate_root
        .into_iter()
        .flat_map(|root| [root.join("src/lib.rs"), root.join("src/main.rs")])
        .collect();
    prod_queue.extend(listed.iter().map(PathBuf::from));
    let mut walked = HashSet::new();

    while let Some(path) = prod_queue.pop() {
        if !path.is_file() || !walked.insert(path.clone()) {
            continue;
        }
        let Ok(src) = std::fs::read_to_string(&path) else {
            continue;
        };
        let Some((test_mods, plain_mods)) = declared_modules(&src) else {
            continue;
        };
        for name in &test_mods {
            test_queue.extend(child_module_candidates(&path, name));
        }
        // Descending through production modules is what finds a declaration
        // the export never mentioned.
        //
        // ⚠ [`child_module_candidates`] resolves `mod name;` by convention and
        // does not read `#[path = "…"]`. A relocated production module would
        // stop the descent for its whole subtree — costing detections, never
        // causing false ones. The workspace contains no `#[path]` today.
        for name in &plain_mods {
            prod_queue.extend(child_module_candidates(&path, name));
        }
    }

    // Once a file is test-only, everything it declares is too — `#[cfg(test)]`
    // is not repeated on the children.
    while let Some(path) = test_queue.pop() {
        if !path.is_file() || !found.insert(path.clone()) {
            continue;
        }
        let Ok(src) = std::fs::read_to_string(&path) else {
            continue;
        };
        let Some((test_mods, plain_mods)) = declared_modules(&src) else {
            continue;
        };
        for name in test_mods.iter().chain(plain_mods.iter()) {
            test_queue.extend(child_module_candidates(&path, name));
        }
    }
    found
}

/// One llvm-cov segment: `[line, col, count, hasCount, isRegionEntry, isGapRegion]`.
struct Seg {
    line: usize,
    count: u64,
    has_count: bool,
    is_region_entry: bool,
    is_gap: bool,
}

fn is_start_of_region(s: &Seg) -> bool {
    !s.is_gap && s.has_count && s.is_region_entry
}

/// Execution count per *mapped* line of one file.
///
/// Ported from LLVM's `LineCoverageIterator` / `LineCoverageStats`
/// (`CoverageMapping.cpp`). Segments mark region **boundaries**, so lines
/// between two boundaries carry the enclosing ("wrapped") region's count and
/// must be filled in; counting only the lines that own a segment undercounts
/// the denominator by 9-26 % across the seven crates measured here.
///
/// ★ **Validated against llvm-cov's own per-line output.** Checked against the
/// `DA:` records of `cargo llvm-cov --lcov` for `sim-types` (409 lines) and
/// `cf-fsu-model` (3152 lines): identical on every file, line for line and
/// count for count.
///
/// That second crate is what caught the seeding rule. `sim-types` agreed under
/// either reading, so one crate was not enough to pin it — the disagreement
/// only appears where a dead branch closes inside a hot function.
///
/// ⚠ That is *not* the same as the `summary` field in the JSON export, which
/// this criterion no longer reads. For `sim/L0/types/src/body.rs` the summary
/// claims 138 lines while llvm-cov's own per-line export emits 136 — the tool
/// disagrees with itself, and the per-line data is the side `llvm-cov show`
/// and lcov report. So the grade's percentage will sit a few tenths off a bare
/// `cargo llvm-cov --summary-only`, for this reason on top of the
/// `#[cfg(test)]` exclusion.
pub(crate) fn mapped_lines(segments: &[serde_json::Value]) -> BTreeMap<usize, u64> {
    let mut out = BTreeMap::new();
    let mut segs: Vec<Seg> = segments
        .iter()
        .filter_map(|s| {
            let a = s.as_array()?;
            Some(Seg {
                line: a.first()?.as_u64()? as usize,
                count: a.get(2)?.as_u64().unwrap_or(0),
                has_count: a.get(3)?.as_bool().unwrap_or(false),
                is_region_entry: a.get(4)?.as_bool().unwrap_or(false),
                is_gap: a
                    .get(5)
                    .and_then(serde_json::Value::as_bool)
                    .unwrap_or(false),
            })
        })
        .collect();
    // llvm-cov emits segments in source order. Sorting makes the walk
    // independent of that promise: out-of-order input would leave `next`
    // unable to advance while `line` climbed, spinning forever. The sort is
    // stable, so within-line column order — which the skipped-region check
    // reads off the first segment — is preserved.
    segs.sort_by_key(|s| s.line);
    let Some(first) = segs.first() else {
        return out;
    };

    let mut wrapped: Option<&Seg> = None;
    let mut group: Vec<&Seg> = Vec::new();
    let mut next = 0usize;
    let mut line = first.line;

    while next < segs.len() {
        if let Some(last) = group.last() {
            wrapped = Some(last);
        }
        group.clear();
        while next < segs.len() && segs[next].line == line {
            group.push(&segs[next]);
            next += 1;
        }

        let entries = group.iter().filter(|s| is_start_of_region(s)).count();
        // A line whose first segment opens a *skipped* region is not code.
        let skipped = group
            .first()
            .is_some_and(|s| !s.has_count && s.is_region_entry);
        let mapped = !skipped && (wrapped.is_some_and(|w| w.has_count) || entries > 0);

        if mapped {
            // Seed from the enclosing region, then let any region opening on
            // this line raise it. A line can *close* a zero-count region while
            // still executing as part of the region that wraps it — the `}` of
            // an `if let` whose body never ran, inside a hot function. Seeding
            // at zero whenever a region opens here reports those as uncovered.
            let mut exec = wrapped.filter(|w| w.has_count).map_or(0, |w| w.count);
            for s in &group {
                if is_start_of_region(s) {
                    exec = exec.max(s.count);
                }
            }
            out.insert(line, exec);
        }
        line += 1;
    }
    out
}

/// Production line coverage for `crate_path`, from one llvm-cov JSON export.
///
/// Files are matched by [`is_own_production_file`]: under
/// `crate_path` by path substring, as the whole-file aggregation did, and NOT
/// under its `tests/`. That second half is newer than the first — integration
/// targets are instrumented so their binaries emit profiles, which puts their
/// source in the export, and counting it would pad both sides with
/// ~100 %-covered test bodies. Same distortion this module strips
/// `#[cfg(test)]` to avoid, arriving by a different route.
pub(crate) fn production_coverage(
    json: &serde_json::Value,
    crate_path: &str,
) -> ProductionCoverage {
    let mut acc = ProductionCoverage::default();
    let Some(files) = json["data"][0]["files"].as_array() else {
        return acc;
    };

    let own: Vec<String> = files
        .iter()
        .filter_map(|f| f["filename"].as_str())
        .filter(|name| is_own_production_file(name, crate_path))
        .map(str::to_string)
        .collect();
    // With no own files there is no root — and nothing to measure either, so
    // the empty result is never consulted. Which own file is used does not
    // matter: every one of them contains `crate_path`, so each cuts back to the
    // same root, unless a path repeats the fragment inside the crate — a shape
    // no crate here has, and one [`crate_root_of`] degrades safely on anyway.
    let crate_root = own.first().and_then(|name| crate_root_of(name, crate_path));
    let test_files = test_only_files(crate_root.as_deref(), &own);

    for file in files {
        let name = file["filename"].as_str().unwrap_or("");
        if !is_own_production_file(name, crate_path) {
            // Integration-test source is this crate's, just not its production
            // code. Report those lines as excluded rather than dropping them
            // silently — same treatment as a `#[cfg(test)] mod` file below,
            // and the same reason: a metric that omits what it left out reads
            // as "everything was measured". Foreign files (a dependency's
            // macro source) are not this crate's at all and stay unmentioned.
            if path_is_under(name, crate_path) {
                if let Some(segments) = file["segments"].as_array() {
                    acc.excluded += mapped_lines(segments).len() as u64;
                }
            }
            continue;
        }
        let Some(segments) = file["segments"].as_array() else {
            continue;
        };

        // A file pulled in by `#[cfg(test)] mod name;` is test code end to end,
        // and carries no attribute of its own to say so.
        if test_files.contains(Path::new(name)) {
            acc.excluded += mapped_lines(segments).len() as u64;
            continue;
        }

        let parsed = std::fs::read_to_string(name)
            .ok()
            .and_then(|src| cfg_test_spans(&src));
        // Recorded per file, not just per crate. With no spans to subtract, an
        // unparsed file keeps its `#[cfg(test)]` lines in the count — and
        // `#[ignore]`d gates are uncovered — so its row overstates how much
        // PRODUCTION code is missing tests, by an amount nothing here can bound.
        // Ranked among the honest rows unmarked, it sends a reader to the wrong
        // file, which is the one thing a triage table must not do.
        let test_lines_counted = parsed.is_none();
        let spans = match parsed {
            Some(spans) => spans,
            None => {
                acc.unparsed.push(name.to_string());
                Vec::new()
            }
        };

        let relative = relative_to_crate(name, crate_path);
        let is_bin = is_bin_target_file(&relative);
        let mut this_file = FileCoverage {
            file: relative,
            covered: 0,
            total: 0,
            test_lines_counted,
            is_bin,
        };
        for (line, count) in mapped_lines(segments) {
            if spans.iter().any(|(a, b)| line >= *a && line <= *b) {
                acc.excluded += 1;
            } else {
                this_file.total += 1;
                this_file.covered += u64::from(count > 0);
            }
        }
        // A file whose every mapped line was `#[cfg(test)]` contributes nothing
        // to the ratio and nothing to write tests against, so it is not a triage
        // row. Its lines are still reported, in `excluded` above.
        if this_file.total > 0 {
            acc.covered += this_file.covered;
            acc.total += this_file.total;
            // Accumulated as a SUBSET of the totals above, not instead of them,
            // so `percent()` is unmoved and only `lib_percent()` reads these.
            if is_bin {
                acc.bin_covered += this_file.covered;
                acc.bin_total += this_file.total;
            }
            acc.files.push(this_file);
        }
    }

    // Biggest win first, path as the tiebreak so the order is a function of the
    // measurement rather than of llvm-cov's file ordering.
    acc.files
        .sort_by(|a, b| b.uncovered().cmp(&a.uncovered()).then(a.file.cmp(&b.file)));
    acc
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `#[cfg(test)]` is not only a module attribute in this workspace — it
    /// appears on free functions, on methods inside production impls, and on
    /// struct fields. A text scan for `mod tests` would miss all three.
    #[test]
    fn spans_cover_every_shape_cfg_test_takes() {
        let src = r#"
pub fn prod() -> i32 { 1 }

#[cfg(test)]
fn helper() -> i32 { 2 }

struct S {
    real: i32,
    #[cfg(test)]
    probe: i32,
}

impl S {
    fn keep(&self) -> i32 { self.real }

    #[cfg(test)]
    fn duplicate(&self) -> i32 { self.real }
}

#[cfg(test)]
mod tests {
    #[test]
    fn t() {}
}
"#;
        let spans = cfg_test_spans(src).expect("parses");
        let covers = |line: usize| spans.iter().any(|(a, b)| line >= *a && line <= *b);

        assert!(covers(5), "free fn under cfg(test)");
        assert!(covers(10), "struct field under cfg(test)");
        assert!(covers(17), "method under cfg(test)");
        assert!(covers(23), "test module");

        // Production lines must survive, or the denominator loses real code.
        assert!(!covers(2), "production fn");
        assert!(!covers(8), "production field");
        assert!(!covers(14), "production method");
    }

    /// `any(test, ...)` items exist in a non-test build, so they are production.
    /// Getting this backwards would silently drop shipped code from the metric.
    #[test]
    fn any_test_is_production_but_all_test_is_not() {
        let any = cfg_test_spans("#[cfg(any(test, feature = \"f\"))]\nfn a() {}\n").unwrap();
        assert!(any.is_empty(), "any(test, ..) must stay in the denominator");

        let all = cfg_test_spans("#[cfg(all(test, unix))]\nfn a() {}\n").unwrap();
        assert_eq!(all.len(), 1, "all(test, ..) is test-only");
    }

    /// A feature whose NAME contains "test" is not a test gate. Matching on the
    /// substring drops shipped code out of the metric silently, which is worse
    /// than any wrong percentage — the crate looks measured and is not.
    #[test]
    fn a_feature_named_like_test_is_not_a_test_gate() {
        let spans =
            cfg_test_spans("#[cfg(all(feature = \"test-fixtures\", unix))]\nfn a() {}\n").unwrap();
        assert!(
            spans.is_empty(),
            "all(feature = \"test-fixtures\", ..) is production, not test-only"
        );
    }

    /// `mod.rs`/`lib.rs` own their own directory; any other file owns a
    /// subdirectory named after itself. Getting this backwards resolves to
    /// nothing, and the test file silently scores as production code.
    #[test]
    fn child_modules_resolve_for_both_owner_shapes() {
        let from_mod = child_module_candidates(Path::new("a/src/parser/mod.rs"), "tests");
        assert_eq!(from_mod[0], Path::new("a/src/parser/tests.rs"));
        assert_eq!(from_mod[1], Path::new("a/src/parser/tests/mod.rs"));

        let from_leaf = child_module_candidates(Path::new("a/src/parser.rs"), "tests");
        assert_eq!(from_leaf[0], Path::new("a/src/parser/tests.rs"));
        assert_eq!(from_leaf[1], Path::new("a/src/parser/tests/mod.rs"));
    }

    /// The gate sits on the *declaration*, so the file it names contains
    /// nothing that identifies it as test code.
    #[test]
    fn non_inline_test_modules_are_separated_from_ordinary_ones() {
        let (test, plain) = declared_modules(
            "#[cfg(test)]\nmod tests;\nmod parser;\n#[cfg(test)]\nmod inline { fn f() {} }\n",
        )
        .unwrap();
        assert_eq!(test, vec!["tests".to_string()]);
        assert_eq!(plain, vec!["parser".to_string()]);
    }

    #[test]
    fn unparseable_source_is_reported_not_guessed() {
        assert!(cfg_test_spans("fn broken( {").is_none());
    }

    fn seg(
        line: u64,
        col: u64,
        count: u64,
        has: bool,
        entry: bool,
        gap: bool,
    ) -> serde_json::Value {
        serde_json::json!([line, col, count, has, entry, gap])
    }

    /// Segments mark region *boundaries*. The lines between an entry and its
    /// end belong to the enclosing region and must be filled in — this is the
    /// bug that made a first attempt undercount every file it touched.
    #[test]
    fn lines_between_two_segments_inherit_the_wrapped_count() {
        let segments = vec![
            seg(10, 5, 7, true, true, false),   // region entry, executed 7×
            seg(14, 6, 0, false, false, false), // region end
        ];
        let lines = mapped_lines(&segments);
        assert_eq!(
            lines.keys().copied().collect::<Vec<_>>(),
            vec![10, 11, 12, 13, 14],
            "every line of the region is mapped, not just the two with segments"
        );
        assert!(lines.values().all(|c| *c == 7));
    }

    /// Validated against llvm-cov's own `DA:` records: a line closing a
    /// zero-count region inside an executing one is covered, not uncovered.
    /// Seeding the count at zero whenever a region opens on the line got this
    /// backwards on 4 lines of cf-fsu-model.
    #[test]
    fn a_line_closing_a_dead_branch_inside_a_live_region_is_covered() {
        let segments = vec![
            seg(10, 5, 7, true, true, false), // live region, executed 7x
            seg(12, 9, 0, true, true, false), // a branch on line 12 that never ran
            seg(12, 10, 0, false, false, false),
        ];
        let lines = mapped_lines(&segments);
        assert_eq!(
            lines.get(&12),
            Some(&7),
            "line 12 still executes as part of the wrapping region"
        );
    }

    #[test]
    fn an_unexecuted_region_is_mapped_but_uncovered() {
        let segments = vec![
            seg(3, 1, 0, true, true, false),
            seg(5, 2, 0, false, false, false),
        ];
        let lines = mapped_lines(&segments);
        assert_eq!(lines.len(), 3);
        assert!(
            lines.values().all(|c| *c == 0),
            "an uncovered region still occupies the denominator"
        );
    }

    /// ★ `tests/` source must be excluded from the crate's own production files.
    ///
    /// Test targets are instrumented so their binaries emit a profile at all
    /// (see [`is_test_target`]), which puts their source in the export. Counting
    /// it would pad both sides of the ratio with ~100 %-covered test bodies —
    /// measured on `cf-geometry`, 89.7 % with them versus 85.7 % without.
    #[test]
    fn integration_test_source_is_not_a_production_file() {
        let cp = "design/cf-geometry";
        assert!(is_own_production_file(
            "/w/cortenforge/design/cf-geometry/src/mesh.rs",
            cp
        ));
        assert!(!is_own_production_file(
            "/w/cortenforge/design/cf-geometry/tests/mesh_tests.rs",
            cp
        ));
        assert!(
            !is_own_production_file("/w/cortenforge/mesh/mesh-repair/src/lib.rs", cp),
            "another crate's source is not this crate's"
        );
    }

    /// ★★ The sibling-prefix regression, and the FAIL-OPEN half of it.
    ///
    /// The check above uses `design/cf-geometry` against `mesh/mesh-repair` —
    /// paths that are not prefixes of one another, so it passed even when the
    /// predicate was a bare `contains`. This one uses a colliding pair, which
    /// is the only kind that could ever have caught the bug.
    ///
    /// The second assertion is the dangerous one: a SIBLING's `tests/` was
    /// counted as this crate's PRODUCTION code, because the exclusion strips
    /// only `<crate_path>/tests/` and `mesh/mesh-io/tests/…` does not contain
    /// `mesh/mesh/tests/`. ~100 %-covered test bodies padded the numerator of
    /// a crate that never ran them — coverage reading HIGHER than the truth.
    #[test]
    fn sibling_crate_sharing_a_path_prefix_is_not_own_production() {
        let cp = "mesh/mesh";
        assert!(
            is_own_production_file("mesh/mesh/src/lib.rs", cp),
            "the crate's own source must still count"
        );
        assert!(
            !is_own_production_file("mesh/mesh-io/src/stl.rs", cp),
            "`mesh/mesh-io` is a sibling of `mesh/mesh`, not part of it"
        );
        assert!(
            !is_own_production_file("mesh/mesh-io/tests/roundtrip.rs", cp),
            "a SIBLING's integration tests are not this crate's production code"
        );
        assert!(
            !is_own_production_file("mesh/mesh/tests/it.rs", cp),
            "the crate's own integration tests are still excluded"
        );
    }

    /// A `crate_path` with a trailing slash must behave identically.
    ///
    /// Caught reviewing the fix above, not the original bug. `path_is_under`
    /// trims only what it is handed, so composing the exclusion as
    /// `format!("{crate_path}/tests")` on `"mesh/mesh/"` yielded
    /// `"mesh/mesh//tests"` — an interior `//` matching nothing, which turned
    /// the tests exclusion OFF and counted the crate's own integration tests
    /// as production. Fail-open, in the fix for a fail-open.
    #[test]
    fn trailing_slash_on_crate_path_does_not_disable_the_tests_exclusion() {
        for cp in ["mesh/mesh", "mesh/mesh/"] {
            assert!(
                is_own_production_file("mesh/mesh/src/lib.rs", cp),
                "own source must count for crate_path {cp:?}"
            );
            assert!(
                !is_own_production_file("mesh/mesh/tests/it.rs", cp),
                "own integration tests must be excluded for crate_path {cp:?}"
            );
            assert!(
                !is_own_production_file("mesh/mesh-io/src/stl.rs", cp),
                "a sibling must be excluded for crate_path {cp:?}"
            );
        }
    }

    #[test]
    fn path_is_under_handles_relative_absolute_and_boundaries() {
        assert!(path_is_under("mesh/mesh/src/lib.rs", "mesh/mesh"));
        assert!(path_is_under(
            "/w/cortenforge/mesh/mesh/src/lib.rs",
            "mesh/mesh"
        ));
        assert!(path_is_under("mesh/mesh/src/lib.rs", "mesh/mesh/"));
        assert!(!path_is_under("mesh/mesh-io/src/lib.rs", "mesh/mesh"));
        assert!(!path_is_under(
            "/w/cortenforge/mesh/mesh-io/src/lib.rs",
            "mesh/mesh"
        ));
        // Not a prefix at a segment boundary: `xmesh/mesh` must not match.
        assert!(!path_is_under("xmesh/mesh/src/lib.rs", "mesh/mesh"));
    }

    #[test]
    fn empty_segment_list_yields_no_lines() {
        assert!(mapped_lines(&[]).is_empty());
    }

    /// ★ Integration-test source reaches the export and must not be counted.
    ///
    /// The predicate is unit-tested next door in `coverage_run`; this pins the
    /// integration — that `production_coverage` actually skips those files
    /// rather than merely being able to classify them. Both files here have
    /// identical segments, so any leak shows up as a doubled denominator.
    #[test]
    fn integration_test_source_is_not_counted_as_production() {
        let segments =
            serde_json::json!([[1, 1, 5, true, true, false], [3, 1, 0, false, false, false]]);
        let json = serde_json::json!({
            "data": [{"files": [
                {"filename": "/w/design/cf-geometry/src/mesh.rs", "segments": segments},
                {"filename": "/w/design/cf-geometry/tests/mesh_tests.rs", "segments": segments},
            ]}]
        });
        let cov = production_coverage(&json, "design/cf-geometry");
        assert_eq!(
            cov.total, 3,
            "only src/mesh.rs's 3 mapped lines belong in the denominator; \
             tests/ source must not double it"
        );
        assert_eq!(cov.covered, 3);
        assert_eq!(
            cov.excluded, 3,
            "the tests/ file's lines must be REPORTED as excluded, not dropped \
             silently — the criterion's own rule about saying what was left out"
        );
    }

    /// Build a throwaway crate directory whose `src/` holds `files`.
    ///
    /// Returns the crate root, so the caller passes exactly what `grade` does.
    fn crate_dir(tag: &str, files: &[(&str, &str)]) -> PathBuf {
        let root = std::env::temp_dir().join(format!(
            "cf-prod-code-{tag}-{}-{:?}",
            std::process::id(),
            std::thread::current().id()
        ));
        let _ = std::fs::remove_dir_all(&root);
        for (name, text) in files {
            let path = root.join("src").join(name);
            std::fs::create_dir_all(path.parent().expect("parent")).expect("mkdir");
            std::fs::write(&path, text).expect("write");
        }
        root
    }

    /// The four `*-benches` crates, exactly as they are on disk: a lib that is
    /// nothing but a module doc comment, with every benchmark in `benches/`.
    #[test]
    fn a_lib_that_is_only_a_doc_comment_declares_no_production_code() {
        let root = crate_dir(
            "doc-only",
            &[(
                "lib.rs",
                "//! Benchmark companion crate for `sim-core`.\n//!\n\
                 //! Bench harnesses live under `benches/`; this crate has no library API.\n",
            )],
        );
        assert!(
            declares_no_production_code(&root),
            "a lib with no items is the empty-crate case the Coverage F was misreading"
        );
        let _ = std::fs::remove_dir_all(&root);
    }

    /// The guard this must not weaken: a crate with real code that produced no
    /// profile is the #770 instrumentation defect, and has to keep failing.
    #[test]
    fn one_production_function_is_enough_to_disqualify_the_empty_verdict() {
        let root = crate_dir("has-fn", &[("lib.rs", "pub fn f() -> i32 { 1 }\n")]);
        assert!(
            !declares_no_production_code(&root),
            "a crate with an instrumentable function must keep reporting the failure"
        );
        let _ = std::fs::remove_dir_all(&root);
    }

    /// A pure re-export facade emits no code of its own, and `mod name;` points
    /// at a file that is walked in its own right.
    #[test]
    fn re_exports_and_module_declarations_are_not_production_code() {
        let root = crate_dir(
            "facade",
            &[("lib.rs", "pub use other::Thing;\nmod inner;\n")],
        );
        assert!(
            declares_no_production_code(&root),
            "`use` and a non-inline `mod` emit nothing themselves"
        );
        let _ = std::fs::remove_dir_all(&root);
    }

    /// ★ The walk is recursive, so a second file is not a blind spot — the
    /// declaration in `lib.rs` emits nothing but the file it names does.
    #[test]
    fn code_in_a_child_module_file_is_found() {
        let root = crate_dir(
            "child",
            &[
                ("lib.rs", "mod inner;\n"),
                ("inner.rs", "pub fn buried() -> i32 { 2 }\n"),
            ],
        );
        assert!(
            !declares_no_production_code(&root),
            "walking only lib.rs would call this crate empty"
        );
        let _ = std::fs::remove_dir_all(&root);
    }

    /// Test-gated items are not production code, so a crate that is nothing but
    /// its own tests is still empty for this purpose.
    #[test]
    fn a_cfg_test_module_is_not_production_code() {
        let root = crate_dir(
            "cfg-test",
            &[(
                "lib.rs",
                "#[cfg(test)]\nmod tests {\n    #[test]\n    fn t() {}\n}\n",
            )],
        );
        assert!(
            declares_no_production_code(&root),
            "#[cfg(test)] code carries no coverage map in a production build"
        );
        let _ = std::fs::remove_dir_all(&root);
    }

    /// ⚠ The load-bearing half of the conservatism: a file this cannot parse
    /// must NOT be read as emptiness, or a syntax error would silently convert a
    /// real instrumentation defect into a pass.
    ///
    /// ★ The companion `lib.rs` is what makes this test able to fail for the
    /// reason it names. A bare unparseable file would also return `false` if the
    /// walk had simply found nothing, and the two causes are indistinguishable
    /// from the return value alone. The doc-only `lib.rs` is a file the walk
    /// must see and accept, so `false` here can only have come from `broken.rs`.
    #[test]
    fn an_unparseable_file_is_never_called_empty() {
        let root = crate_dir(
            "unparseable",
            &[
                ("lib.rs", "//! Empty on purpose.\n"),
                ("broken.rs", "pub fn (((\n"),
            ],
        );
        assert!(
            !declares_no_production_code(&root),
            "a file that cannot be parsed proves nothing about what it declares"
        );

        std::fs::remove_file(root.join("src").join("broken.rs")).expect("rm");
        assert!(
            declares_no_production_code(&root),
            "with only the parseable doc-only file left the verdict must flip — \
             otherwise the assertion above would hold even if the walk saw nothing"
        );
        let _ = std::fs::remove_dir_all(&root);
    }

    /// ★ `#[cfg(test)]` gates free functions too, not only modules.
    ///
    /// The sibling test puts its test code inside a `#[cfg(test)] mod`, where
    /// the module arm answers first and the function arm is never consulted —
    /// so without this case, deleting the function arm entirely changes no test
    /// outcome. Found by mutating it away and watching the suite stay green.
    #[test]
    fn a_cfg_test_free_function_is_not_production_code() {
        let root = crate_dir(
            "cfg-test-fn",
            &[(
                "lib.rs",
                "use std::fmt;\n\n#[cfg(test)]\nfn fixture() -> i32 { 7 }\n",
            )],
        );
        assert!(
            declares_no_production_code(&root),
            "a test-gated free function is test code wherever it sits"
        );
        let _ = std::fs::remove_dir_all(&root);
    }

    /// ★ A symlinked module must be read, not skipped.
    ///
    /// `WalkDir` does not follow links by default, and an unfollowed link is not
    /// a file — so the entry would be filtered out by the same test that skips
    /// directories, and a crate whose implementation sits behind one would be
    /// declared empty. This fails without `follow_links(true)`.
    #[cfg(unix)]
    #[test]
    fn code_behind_a_symlink_is_not_skipped() {
        let root = crate_dir("symlink", &[("lib.rs", "//! Empty on purpose.\n")]);
        let target = root.join("real_impl.rs");
        std::fs::write(&target, "pub fn behind_a_link() -> i32 { 3 }\n").expect("write");
        std::os::unix::fs::symlink(&target, root.join("src").join("linked.rs")).expect("symlink");

        assert!(
            !declares_no_production_code(&root),
            "a symlinked source file holds production code just as a regular one does"
        );
        let _ = std::fs::remove_dir_all(&root);
    }

    /// Two production files, one fully covered and one not at all.
    ///
    /// Shared by the per-file tests so they disagree about one thing at a time.
    /// The paths need not exist: an unreadable file yields no `cfg(test)` spans,
    /// which is the same as a file that has none.
    fn two_file_export() -> serde_json::Value {
        serde_json::json!({
            "data": [{"files": [
                // lines 1-3, executed 5x
                {"filename": "/w/demo/src/covered.rs", "segments": [
                    [1, 1, 5, true, true, false], [3, 1, 0, false, false, false]]},
                // lines 1-4, never executed
                {"filename": "/w/demo/src/cold.rs", "segments": [
                    [1, 1, 0, true, true, false], [4, 1, 0, false, false, false]]},
            ]}]
        })
    }

    /// ★ The per-file rows must reconstruct the crate total exactly.
    ///
    /// Both come from one pass over one set of lines, and that is the whole
    /// license for showing them together: a breakdown that disagreed with the
    /// percentage printed above it would be worse than no breakdown, because a
    /// reader would trust it to decide where to spend an afternoon.
    #[test]
    fn per_file_rows_sum_to_the_crate_total() {
        let cov = production_coverage(&two_file_export(), "demo");

        assert_eq!(cov.total, 7, "3 covered lines + 4 cold ones");
        assert_eq!(cov.covered, 3);
        assert_eq!(
            cov.files.iter().map(|f| f.total).sum::<u64>(),
            cov.total,
            "per-file denominators must sum to the crate denominator"
        );
        assert_eq!(
            cov.files.iter().map(|f| f.covered).sum::<u64>(),
            cov.covered,
            "per-file numerators must sum to the crate numerator"
        );
    }

    /// Worst first — the ranking IS the feature, since triage starts at the top
    /// and the printed table is capped.
    #[test]
    fn files_are_ranked_by_uncovered_lines() {
        let cov = production_coverage(&two_file_export(), "demo");

        let order: Vec<&str> = cov.files.iter().map(|f| f.file.as_str()).collect();
        assert_eq!(
            order,
            vec!["src/cold.rs", "src/covered.rs"],
            "the file with 4 uncovered lines must outrank the one with 0"
        );
        assert_eq!(cov.files[0].uncovered(), 4);
        assert_eq!(cov.files[1].uncovered(), 0);
        assert!((cov.files[1].percent() - 100.0).abs() < 1e-9);
    }

    /// Ties break on path, so two runs over one tree print the same order and a
    /// reordered export cannot masquerade as a changed measurement.
    #[test]
    fn equal_uncovered_counts_break_ties_on_path() {
        // Identical segments, so the only thing separating them is the name.
        let segments =
            serde_json::json!([[1, 1, 0, true, true, false], [2, 1, 0, false, false, false]]);
        let json = serde_json::json!({
            "data": [{"files": [
                {"filename": "/w/demo/src/zebra.rs", "segments": segments},
                {"filename": "/w/demo/src/alpha.rs", "segments": segments},
            ]}]
        });
        let cov = production_coverage(&json, "demo");

        let order: Vec<&str> = cov.files.iter().map(|f| f.file.as_str()).collect();
        assert_eq!(
            order,
            vec!["src/alpha.rs", "src/zebra.rs"],
            "equal uncovered counts must sort by path, not by export order"
        );
    }

    /// ★ Integration-test source is excluded from the ratio, so it must not
    /// appear as a triage row either — sending a reader to write tests for a
    /// test file.
    #[test]
    fn integration_test_source_is_not_a_triage_row() {
        let segments =
            serde_json::json!([[1, 1, 0, true, true, false], [3, 1, 0, false, false, false]]);
        let json = serde_json::json!({
            "data": [{"files": [
                {"filename": "/w/demo/src/lib.rs", "segments": segments},
                {"filename": "/w/demo/tests/it.rs", "segments": segments},
            ]}]
        });
        let cov = production_coverage(&json, "demo");

        let order: Vec<&str> = cov.files.iter().map(|f| f.file.as_str()).collect();
        assert_eq!(
            order,
            vec!["src/lib.rs"],
            "tests/ source is not a fix target"
        );
        assert_eq!(
            cov.excluded, 3,
            "and its lines are still reported, not dropped"
        );
    }

    /// A file whose every mapped line is `#[cfg(test)]` contributes nothing to
    /// the ratio, so it is nothing to go and fix — it must not occupy a row that
    /// a real target could have used.
    ///
    /// ★ The companion production file is what lets this fail for the reason it
    /// names: without it, an empty row list would also result from
    /// `production_coverage` finding no files at all.
    #[test]
    fn a_file_with_only_test_lines_is_not_a_triage_row() {
        let root = crate_dir(
            "triage-cfg-test",
            &[
                (
                    "all_test.rs",
                    "#[cfg(test)]\nmod t {\n    #[test]\n    fn a() {}\n}\n",
                ),
                ("real.rs", "pub fn f() -> i32 {\n    1\n}\n"),
            ],
        );
        let crate_path = root.to_str().expect("utf-8 temp path").to_string();
        let segments =
            serde_json::json!([[1, 1, 0, true, true, false], [5, 1, 0, false, false, false]]);
        let json = serde_json::json!({
            "data": [{"files": [
                {"filename": format!("{crate_path}/src/all_test.rs"), "segments": segments},
                {"filename": format!("{crate_path}/src/real.rs"), "segments": segments},
            ]}]
        });

        let cov = production_coverage(&json, &crate_path);
        let order: Vec<&str> = cov.files.iter().map(|f| f.file.as_str()).collect();
        assert_eq!(
            order,
            vec!["src/real.rs"],
            "an all-#[cfg(test)] file is not a place to write production tests"
        );
        assert_eq!(cov.excluded, 5, "its 5 lines are reported as excluded");
        let _ = std::fs::remove_dir_all(&root);
    }

    /// ★★ A declaration-only `mod.rs` compiles to no regions, so llvm-cov omits
    /// it from `files[]` — and the `#[cfg(test)] mod tests;` it holds goes with
    /// it. A walk seeded only from listed files therefore never learns that
    /// `src/pipeline/tests.rs` is test code, and ranks it as the crate's single
    /// largest production gap.
    ///
    /// That is not a hypothetical: it is how the 2026-08-18 weekly sweep scored
    /// `sim-gpu` at 344/5647 lines with "10 test lines excluded", listing three
    /// test files as its three biggest gaps. All three are declared in
    /// `src/pipeline/mod.rs`, which holds no executable code and so was not
    /// among the 18 files llvm-cov reported for the crate.
    ///
    /// ★ The fixture withholds `src/pipeline/mod.rs` from the export on
    /// purpose. Listing it would let the export-seeded walk find the
    /// declaration too, and the test would pass without exercising the descent
    /// from `lib.rs` that it exists to hold.
    ///
    /// ★ `real.rs` is the negative control: with no companion production file,
    /// an empty row list would equally mean no files were found at all.
    #[test]
    fn a_test_module_declared_in_an_unlisted_mod_file_is_still_test_code() {
        let root = crate_dir(
            "unlisted-mod-decl",
            &[
                ("lib.rs", "pub mod pipeline;\n"),
                (
                    "pipeline/mod.rs",
                    "pub mod real;\n#[cfg(test)]\nmod tests;\n",
                ),
                ("pipeline/real.rs", "pub fn f() -> i32 {\n    1\n}\n"),
                (
                    "pipeline/tests.rs",
                    "#[test]\nfn a() {\n    assert_eq!(super::real::f(), 1);\n}\n",
                ),
            ],
        );
        let crate_path = root.to_str().expect("utf-8 temp path").to_string();
        let segments =
            serde_json::json!([[1, 1, 0, true, true, false], [4, 1, 0, false, false, false]]);
        let json = serde_json::json!({
            "data": [{"files": [
                // `src/pipeline/mod.rs` is absent exactly as llvm-cov leaves it.
                {"filename": format!("{crate_path}/src/pipeline/real.rs"), "segments": segments},
                {"filename": format!("{crate_path}/src/pipeline/tests.rs"), "segments": segments},
            ]}]
        });

        let cov = production_coverage(&json, &crate_path);
        let order: Vec<&str> = cov.files.iter().map(|f| f.file.as_str()).collect();
        assert_eq!(
            order,
            vec!["src/pipeline/real.rs"],
            "a test module reached only through an unlisted `mod.rs` is not production code"
        );
        assert_eq!(cov.excluded, 4, "its 4 lines are reported as excluded");
        let _ = std::fs::remove_dir_all(&root);
    }

    /// ★ `rfind`, not `find`. A checkout under a directory repeating the crate's
    /// own path fragment must resolve to the innermost match — the anchoring
    /// [`relative_to_crate`] already uses. Taking the first occurrence yields a
    /// root outside the crate, whose `src/lib.rs` does not exist, silently
    /// costing the walk its disk seeding and reverting it to export-only.
    #[test]
    fn the_crate_root_is_recovered_from_the_last_occurrence() {
        assert_eq!(
            crate_root_of("/w/sim/L0/gpu/src/pipeline/mod.rs", "sim/L0/gpu"),
            Some(PathBuf::from("/w/sim/L0/gpu"))
        );
        assert_eq!(
            crate_root_of("/build/demo/checkout/demo/src/lib.rs", "demo"),
            Some(PathBuf::from("/build/demo/checkout/demo")),
            "the inner `demo` is the crate; the outer one is a build directory"
        );
        assert_eq!(crate_root_of("/elsewhere/other/src/lib.rs", "demo"), None);
    }

    /// ★ The export's file list is still a seed, and this is the case that needs
    /// it. A binary-only crate has no `src/lib.rs` to descend from, so the only
    /// route to `src/bin/cli.rs` is that llvm-cov listed it — which it does,
    /// because a binary root holds `fn main` and therefore carries regions.
    ///
    /// ★ Without the companion production file the empty row list would equally
    /// mean nothing was found at all.
    #[test]
    fn a_test_module_declared_in_a_binary_root_is_still_test_code() {
        let root = crate_dir(
            "bin-root-decl",
            &[
                ("bin/cli.rs", "#[cfg(test)]\nmod cli_tests;\nfn main() {}\n"),
                ("bin/cli/cli_tests.rs", "#[test]\nfn a() {}\n"),
                ("real.rs", "pub fn f() -> i32 {\n    1\n}\n"),
            ],
        );
        let crate_path = root.to_str().expect("utf-8 temp path").to_string();
        let segments =
            serde_json::json!([[1, 1, 0, true, true, false], [4, 1, 0, false, false, false]]);
        let json = serde_json::json!({
            "data": [{"files": [
                {"filename": format!("{crate_path}/src/bin/cli.rs"), "segments": segments},
                {"filename": format!("{crate_path}/src/bin/cli/cli_tests.rs"), "segments": segments},
                {"filename": format!("{crate_path}/src/real.rs"), "segments": segments},
            ]}]
        });

        let cov = production_coverage(&json, &crate_path);
        let rows: Vec<&str> = cov.files.iter().map(|f| f.file.as_str()).collect();
        assert!(
            !rows.contains(&"src/bin/cli/cli_tests.rs"),
            "a test module declared in a binary root is not production code; rows were {rows:?}"
        );
        assert!(rows.contains(&"src/real.rs"), "rows were {rows:?}");
        // 6, not 4: `src/bin/cli.rs` IS in the export, so the two lines of its
        // own `#[cfg(test)] mod cli_tests;` declaration are excluded by the span
        // walk alongside the four lines of the file that declaration gates. The
        // sibling fixtures withhold their declaring file, so only this one shows
        // both routes to `excluded` at once.
        assert_eq!(cov.excluded, 6, "4 gated lines plus 2 declaration lines");
        let _ = std::fs::remove_dir_all(&root);
    }

    /// ★ `src/main.rs` is seeded from disk, not from the export. The fixture
    /// withholds it from `files[]` — a binary whose object never reached the
    /// merge leaves nothing to list — so only the disk seed can open it and find
    /// the declaration it holds.
    #[test]
    fn a_test_module_declared_in_an_unlisted_main_is_still_test_code() {
        let root = crate_dir(
            "unlisted-main-decl",
            &[
                (
                    "main.rs",
                    "#[cfg(test)]\nmod tests;\nmod real;\nfn main() {}\n",
                ),
                ("real.rs", "pub fn f() -> i32 {\n    1\n}\n"),
                ("tests.rs", "#[test]\nfn a() {}\n"),
            ],
        );
        let crate_path = root.to_str().expect("utf-8 temp path").to_string();
        let segments =
            serde_json::json!([[1, 1, 0, true, true, false], [4, 1, 0, false, false, false]]);
        let json = serde_json::json!({
            "data": [{"files": [
                // `src/main.rs` is deliberately absent from the export.
                {"filename": format!("{crate_path}/src/real.rs"), "segments": segments},
                {"filename": format!("{crate_path}/src/tests.rs"), "segments": segments},
            ]}]
        });

        let cov = production_coverage(&json, &crate_path);
        let rows: Vec<&str> = cov.files.iter().map(|f| f.file.as_str()).collect();
        assert_eq!(
            rows,
            vec!["src/real.rs"],
            "a declaration in an unlisted `main.rs` still marks its file test code"
        );
        assert_eq!(cov.excluded, 4, "its 4 lines are reported as excluded");
        let _ = std::fs::remove_dir_all(&root);
    }

    /// ★ Once a file is test-only its children are too, and they do not repeat
    /// `#[cfg(test)]` — a helper a test file pulls in with a plain `mod helpers;`
    /// carries nothing at all to identify itself. Only chaining the plain modules
    /// through the test walk marks it, and dropping that chain silently promotes
    /// every such helper to production code.
    #[test]
    fn a_helper_module_below_a_test_module_is_test_code_too() {
        let root = crate_dir(
            "test-helper-chain",
            &[
                ("lib.rs", "#[cfg(test)]\nmod tests;\npub mod real;\n"),
                ("tests.rs", "mod helpers;\n#[test]\nfn a() {}\n"),
                ("tests/helpers.rs", "pub fn fixture() -> i32 {\n    1\n}\n"),
                ("real.rs", "pub fn f() -> i32 {\n    1\n}\n"),
            ],
        );
        let crate_path = root.to_str().expect("utf-8 temp path").to_string();
        let segments =
            serde_json::json!([[1, 1, 0, true, true, false], [4, 1, 0, false, false, false]]);
        let json = serde_json::json!({
            "data": [{"files": [
                {"filename": format!("{crate_path}/src/real.rs"), "segments": segments},
                {"filename": format!("{crate_path}/src/tests/helpers.rs"), "segments": segments},
            ]}]
        });

        let cov = production_coverage(&json, &crate_path);
        let rows: Vec<&str> = cov.files.iter().map(|f| f.file.as_str()).collect();
        assert_eq!(
            rows,
            vec!["src/real.rs"],
            "a helper below a `#[cfg(test)]` module is test code, not a place to write tests"
        );
        assert_eq!(cov.excluded, 4, "its 4 lines are reported as excluded");
        let _ = std::fs::remove_dir_all(&root);
    }

    /// ★ A file whose test lines could not be subtracted must say so on its own
    /// row, not only in the crate-level count.
    ///
    /// With no spans to subtract, an unparsed file keeps its `#[cfg(test)]`
    /// lines — and `#[ignore]`d gates read as uncovered — so its row overstates
    /// the gap and can outrank every honest one. The crate report says how many
    /// files this hit; only the row can say which, and the row is what a reader
    /// acts on.
    ///
    /// ★ The parseable companion is the negative control: it proves the flag
    /// tracks parseability rather than being pinned true, which is exactly what
    /// every fixture using a non-existent path would otherwise show.
    #[test]
    fn a_file_whose_test_lines_could_not_be_subtracted_is_marked() {
        let root = crate_dir(
            "triage-unparsed",
            &[
                ("broken.rs", "pub fn (((\n"),
                ("fine.rs", "pub fn f() -> i32 {\n    1\n}\n"),
            ],
        );
        let crate_path = root.to_str().expect("utf-8 temp path").to_string();
        let segments =
            serde_json::json!([[1, 1, 0, true, true, false], [3, 1, 0, false, false, false]]);
        let json = serde_json::json!({
            "data": [{"files": [
                {"filename": format!("{crate_path}/src/broken.rs"), "segments": segments},
                {"filename": format!("{crate_path}/src/fine.rs"), "segments": segments},
            ]}]
        });

        let cov = production_coverage(&json, &crate_path);
        let flags: Vec<(&str, bool)> = cov
            .files
            .iter()
            .map(|f| (f.file.as_str(), f.test_lines_counted))
            .collect();
        assert_eq!(
            flags,
            vec![("src/broken.rs", true), ("src/fine.rs", false)],
            "only the unparseable file's row may claim its test lines were counted"
        );
        assert_eq!(
            cov.unparsed.len(),
            1,
            "and the crate-level report still names it"
        );
        let _ = std::fs::remove_dir_all(&root);
    }

    /// ★ `rfind`, not `find`. A checkout living under a directory that repeats
    /// the crate path would otherwise keep most of the absolute path in every
    /// row, which is exactly the noise relativising exists to remove.
    #[test]
    fn the_crate_root_is_stripped_from_its_last_occurrence() {
        assert_eq!(
            relative_to_crate("/w/cortenforge/sim/L0/core/src/foo.rs", "sim/L0/core"),
            "src/foo.rs"
        );
        assert_eq!(
            relative_to_crate("/home/sim/L0/core/co/sim/L0/core/src/foo.rs", "sim/L0/core"),
            "src/foo.rs",
            "a repeated crate path must resolve to the innermost root"
        );
    }

    /// The fallback is unreachable behind `is_own_production_file`, but must not
    /// panic or truncate if it is ever reached — an unshortened path is readable,
    /// a sliced one is wrong.
    #[test]
    fn a_path_outside_the_crate_is_returned_whole() {
        assert_eq!(
            relative_to_crate("/w/other/src/foo.rs", "demo"),
            "/w/other/src/foo.rs"
        );
    }

    /// A crate with no `src/` at all gets "(no src/)" from a different
    /// criterion; this one must not also claim it.
    #[test]
    fn a_missing_src_directory_is_not_this_verdict() {
        let root = crate_dir("no-src", &[]);
        std::fs::create_dir_all(&root).expect("mkdir");
        assert!(!declares_no_production_code(&root));
        let _ = std::fs::remove_dir_all(&root);
    }

    // === binary-target classification ===

    #[test]
    fn cargos_three_binary_conventions_are_recognised() {
        assert!(is_bin_target_file("src/main.rs"));
        assert!(is_bin_target_file("src/bin/tool.rs"));
        assert!(is_bin_target_file("src/bin/tool/main.rs"));
    }

    /// ★ The complement, asserted separately: a marker that fires on library
    /// code would pull real lines out of the reported library figure, which is
    /// the direction that can manufacture a false pass.
    #[test]
    fn library_sources_are_not_binary_targets() {
        assert!(!is_bin_target_file("src/lib.rs"));
        assert!(!is_bin_target_file("src/sequence.rs"));
        assert!(!is_bin_target_file("src/ui/panel.rs"));
        // A module of a binary, reached by `mod helper;` from src/bin/tool/.
        // Documented as an accepted miss — it counts binary code against the
        // library, which can only understate.
        assert!(!is_bin_target_file("src/bin/tool/helper.rs"));
    }

    /// The match is anchored at the start of the crate-relative path, so a
    /// `src/main.rs` sitting under some other prefix is not this crate's
    /// binary root.
    ///
    /// ⚠ The obvious version of this test — feeding it an absolute path —
    /// asserts nothing: the function takes the output of
    /// [`relative_to_crate`], and an absolute path fails every arm for
    /// reasons that have nothing to do with anchoring. These inputs are the
    /// shape it really receives.
    #[test]
    fn the_binary_root_match_is_anchored_at_the_crate_root() {
        assert!(!is_bin_target_file("vendor/src/main.rs"));
        assert!(!is_bin_target_file("crates/inner/src/main.rs"));
        assert!(!is_bin_target_file("nested/src/bin/tool.rs"));
    }

    fn bin_and_lib_export() -> serde_json::Value {
        serde_json::json!({
            "data": [{"files": [
                // src/lib.rs: lines 1-3, all executed.
                {"filename": "/w/demo/src/lib.rs", "segments": [
                    [1, 1, 5, true, true, false], [3, 1, 0, false, false, false]]},
                // src/main.rs: lines 1-6 inclusive, never executed.
                {"filename": "/w/demo/src/main.rs", "segments": [
                    [1, 1, 0, true, true, false], [6, 1, 0, false, false, false]]},
            ]}]
        })
    }

    /// ★★ The property that makes this change gate-neutral: binary lines are
    /// recorded as a SUBSET of the totals, so `percent()` returns exactly what
    /// it returned before the split existed. If this ever fails, adding the
    /// breakdown silently re-graded the workspace.
    #[test]
    fn binary_lines_are_recorded_without_moving_the_graded_percentage() {
        let cov = production_coverage(&bin_and_lib_export(), "demo");

        assert_eq!(cov.total, 9, "3 library lines + 6 binary ones");
        assert_eq!(cov.covered, 3);
        assert_eq!(
            cov.percent().expect("measured"),
            100.0 * 3.0 / 9.0,
            "the graded figure still spans library AND binary lines"
        );
        assert_eq!(cov.bin_total, 6);
        assert_eq!(cov.bin_covered, 0);
    }

    #[test]
    fn the_library_only_figure_removes_binary_lines_from_both_sides() {
        let cov = production_coverage(&bin_and_lib_export(), "demo");

        assert_eq!(
            cov.lib_percent().expect("has library lines"),
            100.0,
            "3 of 3 library lines are covered; the cold main.rs must not dilute it"
        );
    }

    /// A crate whose only production code IS a binary has no library figure to
    /// report — `None`, not a division by zero dressed up as 0 %.
    #[test]
    fn a_binary_only_crate_has_no_library_percentage() {
        let json = serde_json::json!({
            "data": [{"files": [
                {"filename": "/w/demo/src/main.rs", "segments": [
                    [1, 1, 0, true, true, false], [4, 1, 0, false, false, false]]},
            ]}]
        });
        let cov = production_coverage(&json, "demo");

        assert_eq!(cov.total, 4);
        assert_eq!(cov.bin_total, 4);
        assert!(cov.lib_percent().is_none());
        assert!(cov.percent().is_some(), "the graded figure still exists");
    }

    /// The flag has to survive onto the row, because the triage table is where
    /// a reader learns that a large uncovered count is a binary entry point
    /// rather than a gap they can close.
    #[test]
    fn the_triage_row_for_a_binary_root_is_marked() {
        let cov = production_coverage(&bin_and_lib_export(), "demo");

        let main = cov
            .files
            .iter()
            .find(|f| f.file == "src/main.rs")
            .expect("main.rs row");
        assert!(main.is_bin);
        let lib = cov
            .files
            .iter()
            .find(|f| f.file == "src/lib.rs")
            .expect("lib.rs row");
        assert!(!lib.is_bin);
    }

    // === the bar, the margin, and the letter — one integer comparison ===

    fn measured(covered: u64, total: u64) -> ProductionCoverage {
        ProductionCoverage {
            covered,
            total,
            ..Default::default()
        }
    }

    /// The three ways of asking "is this crate above the bar" have to agree,
    /// or a report can print a letter and a margin that contradict it.
    ///
    /// Exhaustive over every crate size a workspace crate plausibly has and
    /// every covered count within it, at all four grade bands — this is
    /// cheaper than reasoning about which sizes are the interesting ones, and
    /// it does not go stale when a band moves.
    #[test]
    fn meets_the_bar_and_being_at_or_above_it_are_the_same_question() {
        for total in 1..=300u64 {
            for covered in 0..=total {
                let m = measured(covered, total);
                for percent in [40u64, 60, 75, 90] {
                    let meets = m.meets(percent);
                    assert_eq!(
                        meets,
                        covered >= m.bar(percent),
                        "meets vs bar at {covered}/{total} @ {percent}%"
                    );
                    assert_eq!(
                        meets,
                        m.margin_lines(percent).expect("has lines") >= 0,
                        "meets vs margin sign at {covered}/{total} @ {percent}%"
                    );
                }
            }
        }
    }

    /// ⚠ The regression control for the change that introduced all of this:
    /// the letter used to come from `100.0 * covered / total >= 75.0` on an
    /// `f64`, and now comes from `covered * 100 >= 75 * total`. Every crate's
    /// grade had to stay exactly where it was.
    ///
    /// Checked against the `f64` form directly rather than argued from
    /// mantissa widths — the argument is sound for these magnitudes and is
    /// still not evidence. Real crate sizes are swept alongside the small ones
    /// because the two forms diverge, if they ever do, only where the quotient
    /// is least representable.
    #[test]
    fn the_integer_bar_grades_identically_to_the_float_it_replaced() {
        let sizes = (1..=300u64).chain([807, 879, 1082, 1622, 9146]);
        for total in sizes {
            // Every covered count for a small crate; a window around each bar
            // for a large one, where a disagreement would have to live.
            let candidates: Vec<u64> = if total <= 300 {
                (0..=total).collect()
            } else {
                [40u64, 60, 75, 90]
                    .iter()
                    .flat_map(|p| {
                        let bar = (p * total) / 100;
                        (bar.saturating_sub(3)..=(bar + 3).min(total)).collect::<Vec<_>>()
                    })
                    .collect()
            };
            for covered in candidates {
                let m = measured(covered, total);
                let float = m.percent().expect("has lines");
                for percent in [40u64, 60, 75, 90] {
                    assert_eq!(
                        m.meets(percent),
                        float >= percent as f64,
                        "integer/float disagree at {covered}/{total} @ {percent}%"
                    );
                }
            }
        }
    }

    /// ★ The mutant this is shaped to kill: `bar` using `/` instead of
    /// `div_ceil`.
    ///
    /// `sim-soft` on 2026-08-29 measured 9146 production lines, and
    /// `75 × 9146 / 100` is 6859.5 — so truncating division puts the bar at
    /// 6859, one line low. A crate at exactly 6859 covered lines is at
    /// 74.99 % and grades `B`; a truncating `bar` would report it as sitting
    /// exactly ON the A bar with zero margin. Both faces asserted, because a
    /// bar that is merely *high* is the opposite defect and just as wrong.
    #[test]
    fn the_bar_is_the_first_line_that_meets_it_not_the_last_that_misses() {
        let total = 9146;
        assert_eq!(measured(0, total).bar(75), 6860, "75 % of 9146 rounds UP");

        let under = measured(6859, total);
        assert!(!under.meets(75), "6859/9146 is 74.99 %, not 75 %");
        assert_eq!(under.margin_lines(75), Some(-1));

        let at = measured(6860, total);
        assert!(at.meets(75));
        assert_eq!(at.margin_lines(75), Some(0));
    }

    /// A crate with nothing to measure has no margin to report — the same
    /// answer [`ProductionCoverage::percent`] gives, so the two cannot
    /// disagree about whether a measurement happened.
    #[test]
    fn a_crate_with_no_production_lines_has_no_margin() {
        let empty = measured(0, 0);
        assert_eq!(empty.percent(), None);
        assert_eq!(empty.margin_lines(75), None);
    }

    /// The floor has to cover BOTH observations it was sized from, and the
    /// proportional term has to be what covers the large one — a flat 6 would
    /// pass the first assertion and fail the second.
    #[test]
    fn the_noise_floor_covers_both_measured_spreads() {
        // cf-codesign, 2026-08-16: 6 lines of drift on 1622 total.
        assert!(
            measured(1410, 1622).noise_floor_lines() >= 6,
            "cf-codesign's observed 6-line drift must fall inside the band"
        );
        // sim-soft, 2026-08-29: 8198–8216 on 9146 total.
        assert!(
            measured(8201, 9146).noise_floor_lines() >= 18,
            "sim-soft's observed 18-line drift must fall inside the band"
        );
        // The floor, for a crate too small for 0.4 % to reach it.
        assert_eq!(measured(300, 400).noise_floor_lines(), 6);
        // ⚠ Negative control for the floor itself: without the proportional
        // term this would also be 6, and the assertion above would still pass.
        assert!(
            measured(8201, 9146).noise_floor_lines() > 6,
            "a large crate must get a wider band than the flat floor"
        );
    }
}
