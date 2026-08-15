//! Self-test: every release-only gate still reaches some CI job.
//!
//! ⚠ **What this asserts is [`LONG_ABOUT`]** — the text `cargo xtask
//! release-gates --help` prints, and the one place that definition lives (see
//! its doc for why it is a constant). This header carries only the **why**,
//! which a source reader needs and a CLI user does not.
//!
//! # Why coverage has two paths, not one
//!
//! Modelling only the `--test` path is the obvious mistake, and this module
//! shipped it in draft: it reported `mesh-printability`'s `stress_inputs` as
//! unrun when `tests-release` runs that package wholesale. A check that cries
//! wolf gets disabled, so the whole-package path is modelled too — see
//! [`wholesale_release_packages`] for why it must be read from one anchored job
//! rather than from every `crates:` line in the file.
//!
//! # That the hazard is real
//!
//! `#747` found four PRs' worth of reduced-basis gates (`R1.0`–`R1.3`) that had
//! never once executed: green meant *skipped*. Nothing in the tree could have
//! told anyone.
//!
//! # Which direction is actually unguarded
//!
//! Only one of the two *directions* — not to be read against the two coverage
//! paths above. `cargo test --test <name>` on a name that does not exist
//! is a **hard error** (`no test target named …`), so a renamed or deleted test
//! whose `--test` entry was left behind fails CI loudly and needs no help from
//! this module. ⚠ Do not "strengthen" this check by also asserting every listed
//! name exists — cargo already does, at the only moment it matters.
//!
//! The silent direction is the other one: a release-only test that is **added**
//! and reached by neither path. It compiles, the debug shards skip it, no
//! release job picks it up, and it is dead weight that reads as coverage. This
//! module fails on that.
//!
//! # Why a command and not only a unit test
//!
//! Same reason `licensed_gates` gives, and the same trap: `cargo test -p xtask`
//! runs only when xtask is in the PR's affected set. A PR adding
//! `sim/L0/soft/tests/foo.rs` does not touch xtask, so a test-only check would
//! fire on `main`, a merge too late — on precisely the PRs it exists to catch.
//! It runs as a step in the **deliberately unscoped** `licensed-gates` job,
//! which already exists for this exact reason and is a source scan with no
//! build.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

use anyhow::{bail, Result};
use owo_colors::OwoColorize;

use crate::licensed_gates::{owning_package, source_files, target_of, Target};

/// Long help for `cargo xtask release-gates`.
///
/// ★ **The single statement of what this command asserts.** `main.rs` points
/// clap here rather than restating it, and the module header points here rather
/// than restating it — so there is one copy, not three.
///
/// That took two attempts, which is the useful part. The first drift was a clap
/// doc comment in `main.rs` that kept the superseded one-path wording after the
/// header was corrected, so `--help` served the misconception this module exists
/// to prevent. Deleting it *looked* like the fix, but the same explanation still
/// sat in both the header and here — the copy had moved from another file into
/// this one, where it was merely easier to notice. The header now carries only
/// rationale.
///
/// ⇒ **De-duplication means one of the copies stops existing**, not that they
/// end up adjacent.
pub(crate) const LONG_ABOUT: &str = "\
Assert every release-only gate still reaches some CI job.

A test written `#[cfg_attr(debug_assertions, ignore = \"…\")]` is skipped by every \
debug job by design, so it executes only where a release job picks it up — either \
by naming its binary (`--test <name>`) or by running its whole package. Miss both \
and it runs NOWHERE while CI stays green; #747 found four PRs' worth of gates in \
that state.

Derived from the source tree, so a gate added today is checked today. Needs no \
build and no licensed assets.";

/// The merge gate. A release-only test named only by `scheduled.yml` still
/// lets a PR merge without ever running it, so nightly coverage does not count.
const WORKFLOW: &str = ".github/workflows/quality-gate.yml";

/// The job whose matrix names packages run WHOLESALE in release.
///
/// `tests-release` runs `cargo nextest run --release $PKGS` over its `crates:`
/// matrix, and a whole-package release run executes that package's release-only
/// tests without naming any binary. So membership there is the **second** way a
/// gate can be covered, and a check that knew only about `--test` would report
/// every such gate as unrun. It did, on its first run, for
/// `mesh-printability`'s `stress_inputs`.
const RELEASE_JOB: &str = "tests-release";

/// Release-only test binaries deliberately absent from CI altogether.
///
/// **Empty today**, and that is not an oversight — it exists so a future
/// too-expensive-to-run gate has somewhere honest to go instead of quietly
/// vanishing from the inventory. [`check`] asserts every name here still exists
/// as a release-only binary, so it cannot rot into a silent exemption.
///
/// ⚠ `tet10_lame_decision` is **not** here, though its 7.16 GB RSS is exactly
/// the shape of reason this list is for. It carries an *unconditional*
/// `#[ignore]` — its own docs say "*not* `#[cfg_attr(debug_assertions,
/// ignore)]`" — so it is never release-only and never enters this survey. Adding
/// it was the first draft's mistake, and this module's own rot check caught it.
/// Modelled on `licensed_gates::IGNORED_FOR_OTHER_REASONS` and `workflow_gate`'s
/// `UNGATED_JOBS`.
const NOT_IN_ANY_CI_LIST: &[(&str, &str)] = &[];

/// Whether an attribute is `#[cfg_attr(debug_assertions, ignore …)]` — the
/// marker that makes a test release-only.
///
/// ⚠ Matches on the token text containing both `debug_assertions` and `ignore`,
/// which also matches the **inverted** form `#[cfg_attr(not(debug_assertions),
/// ignore)]` — a *debug*-only test, the opposite thing. No such attribute exists
/// in this workspace, and the misclassification would be a **false alarm** (a
/// loud "reaches no job" failure on a test that is release-ignored by design),
/// not a silent miss. Deliberately not guarded: the obvious guard — reject any
/// `not` — would also reject `all(debug_assertions, not(feature = "x"))`, which
/// IS release-only, trading a loud wrong answer for a quiet one. Narrow the
/// match properly if the inverted form ever appears.
fn is_release_only(attr: &syn::Attribute) -> bool {
    if !attr.path().is_ident("cfg_attr") {
        return false;
    }
    match &attr.meta {
        syn::Meta::List(list) => {
            let tokens = list.tokens.to_string();
            tokens.contains("debug_assertions") && tokens.contains("ignore")
        }
        _ => false,
    }
}

/// Names of `#[test]` functions carrying the release-only marker, descending
/// into inline modules so `mod tests { … }` is covered.
fn release_only_tests(items: &[syn::Item], out: &mut Vec<String>) {
    for item in items {
        match item {
            syn::Item::Fn(f) => {
                let is_test = f.attrs.iter().any(|a| a.path().is_ident("test"));
                if is_test && f.attrs.iter().any(is_release_only) {
                    out.push(f.sig.ident.to_string());
                }
            }
            syn::Item::Mod(m) => {
                if let Some((_, items)) = &m.content {
                    release_only_tests(items, out);
                }
            }
            _ => {}
        }
    }
}

/// Every `--test <name>` named anywhere in a workflow file.
///
/// ⚠ Deliberately scans the whole text rather than line-anchored continuations.
/// Both spellings are live in this repo — the continuation form
/// (`--test foo \` on its own line) and the inline form (`cargo test --release
/// -p sim-soft --test bonded_layer_indentation`). A line-anchored pattern reads
/// the second as absent, which is exactly the false alarm this module must not
/// raise: it was made by hand while writing this, and reported a covered gate as
/// missing.
pub(crate) fn listed_targets(yaml: &str) -> BTreeSet<String> {
    // ⚠ Comment lines are dropped first, and that direction matters more than it
    // looks. A stray `--test some_gate` in prose would register as coverage the
    // gate does not have — a FALSE NEGATIVE, i.e. this module silently ceasing
    // to protect, which is worse than a false alarm. Three `--test` mentions
    // already sit in this workflow's comments; none happens to be followed by an
    // identifier today, and "happens to" is not a guarantee worth relying on.
    let body: String = yaml
        .lines()
        .filter(|l| !l.trim_start().starts_with('#'))
        .collect::<Vec<_>>()
        .join("\n");
    let mut listed = BTreeSet::new();
    for (idx, _) in body.match_indices("--test") {
        let rest = &body[idx + "--test".len()..];
        // Skip the separator, which may be spaces, a line continuation, or both.
        let name: String = rest
            .trim_start_matches([' ', '\t', '\\', '\n', '\r'])
            .chars()
            .take_while(|c| c.is_ascii_alphanumeric() || *c == '_')
            .collect();
        if !name.is_empty() {
            listed.insert(name);
        }
    }
    listed
}

/// Packages whose whole test suite runs in release, from [`RELEASE_JOB`]'s
/// `crates:` matrix.
///
/// ⚠ Anchored to that one job block on purpose. `crates:` also appears in the
/// **debug** shards' matrix — including `sim-soft`, the very package whose gates
/// are cherry-picked by `--test` — so unioning every `crates:` line in the file
/// would mark sim-soft wholesale-covered and make this whole check vacuous.
pub(crate) fn wholesale_release_packages(yaml: &str) -> BTreeSet<String> {
    let mut packages = BTreeSet::new();
    let mut inside = false;
    for raw in yaml.lines() {
        let line = raw.trim_end();
        // Same false-negative reasoning as `listed_targets`: a commented-out
        // `crates:` line must not confer coverage.
        if line.trim_start().starts_with('#') {
            continue;
        }
        // A top-level job key: exactly two spaces, then `name:`.
        if let Some(rest) = line.strip_prefix("  ") {
            if !rest.starts_with(' ') && rest.ends_with(':') && !rest.contains(' ') {
                inside = rest.trim_end_matches(':') == RELEASE_JOB;
                continue;
            }
        }
        if !inside {
            continue;
        }
        if let Some((_, value)) = line.split_once("crates:") {
            // Skip an expression like `${{ … }}`; only bare name lists count.
            if value.contains("${{") {
                continue;
            }
            packages.extend(value.split_whitespace().map(str::to_string));
        }
    }
    packages
}

/// One pass over the workspace's integration tests.
///
/// ⚠ `unreadable` is the load-bearing field. A `tests/*.rs` this could not read
/// or parse holds an **unknown** number of release-only gates, not zero — and
/// swallowing it would let a syntax error the scanner cannot handle read as "no
/// gates here", silently un-protecting the file. The aggregate empty-survey
/// guard below does not catch that: it is a *partial* blindness, and the total
/// stays plausible. Same lesson `licensed_gates::Survey` records.
/// ⚠ **What this does NOT cover.** `found` is keyed by binary name alone, while
/// a `--test` entry is package-scoped (`-p <pkg> --test <name>`). Two packages
/// each owning a `tests/smoke.rs` would collide, and coverage of one could read
/// as coverage of the other. No two packages share a test-binary name today
/// (checked: the only repeated stem is `mod`, from `tests/common/mod.rs`
/// helpers, which are not binaries and are excluded by `target_of`), so this is
/// latent rather than live — but it is the shape of hole to widen the key for if
/// it ever becomes live.
#[derive(Default)]
struct Survey {
    /// Release-only test binaries: binary name → owning package.
    found: BTreeMap<String, String>,
    /// Integration-test files that could not be read or parsed.
    unreadable: Vec<PathBuf>,
}

fn survey(root: &Path) -> Survey {
    let mut survey = Survey::default();
    for path in source_files(root) {
        let Some((crate_root, package)) = owning_package(&path) else {
            continue;
        };
        let Some(Target::Integration(binary)) = target_of(&crate_root, &path) else {
            continue;
        };
        let parsed = std::fs::read_to_string(&path)
            .ok()
            .and_then(|text| syn::parse_file(&text).ok());
        let Some(file) = parsed else {
            survey.unreadable.push(path);
            continue;
        };
        let mut tests = Vec::new();
        release_only_tests(&file.items, &mut tests);
        if !tests.is_empty() {
            survey.found.insert(binary, package);
        }
    }
    survey
}

/// Assert every release-only test binary is reached by [`WORKFLOW`] — named by
/// `--test` or swept up by a whole-package release run — and that the allowlist
/// has not rotted. Needs no build and no licensed assets.
pub fn check() -> Result<()> {
    check_at(Path::new("."))
}

/// [`check`], rooted at an explicit workspace directory.
///
/// ⚠ Split out so the unit test can point at the workspace **without**
/// `set_current_dir`. Rust runs a binary's tests on parallel threads in one
/// process, so changing the working directory is a global mutation that can
/// break whichever sibling test happens to read a relative path at that moment —
/// a flake that would appear unrelated to either test.
pub(crate) fn check_at(root: &Path) -> Result<()> {
    let Survey { found, unreadable } = survey(root);

    // A file this could not parse has an UNKNOWN number of release-only gates.
    // Reporting success over it would be the partial-blindness version of the
    // empty-workspace failure guarded below.
    if !unreadable.is_empty() {
        let list: Vec<String> = unreadable.iter().map(|p| p.display().to_string()).collect();
        bail!(
            "could not read or parse {} integration-test file(s), so their release-only \
             gates are UNKNOWN rather than absent:\n  {}",
            unreadable.len(),
            list.join("\n  ")
        );
    }

    // ★ A broken scanner must not read like a clean workspace. `licensed_gates`
    // learned this the hard way — a bad manifest parse once made it report "0
    // gates across 0 crates" and look perfectly healthy. An empty result here
    // means the walk or the parse broke, never that the invariant holds.
    if found.is_empty() {
        bail!(
            "found NO release-only tests anywhere in the workspace. This repo has \
             several, so the scan is broken rather than the invariant satisfied — \
             refusing to report success."
        );
    }

    let workflow = root.join(WORKFLOW);
    let yaml = std::fs::read_to_string(&workflow)
        .map_err(|e| anyhow::anyhow!("cannot read {}: {e}", workflow.display()))?;
    let listed = listed_targets(&yaml);
    if listed.is_empty() {
        bail!("parsed NO `--test` entries out of {WORKFLOW} — the parser is broken");
    }

    let wholesale = wholesale_release_packages(&yaml);
    if wholesale.is_empty() {
        bail!("parsed NO packages out of the `{RELEASE_JOB}` job matrix — the parser is broken");
    }

    let exempt: BTreeSet<&str> = NOT_IN_ANY_CI_LIST.iter().map(|(n, _)| *n).collect();

    // Two ways to be covered: named by `--test`, or in a package whose whole
    // suite runs in release. Only the first was modelled at first draft, and the
    // second is what `mesh-printability` uses.
    let missing: Vec<_> = found
        .iter()
        .filter(|(binary, package)| {
            !listed.contains(*binary)
                && !wholesale.contains(*package)
                && !exempt.contains(binary.as_str())
        })
        .collect();

    // A stale allowlist entry is its own defect: it either names a binary that
    // no longer exists, or one that is no longer release-only, and either way it
    // is an exemption nobody is checking.
    let stale: Vec<&str> = NOT_IN_ANY_CI_LIST
        .iter()
        .map(|(n, _)| *n)
        .filter(|n| !found.contains_key(*n))
        .collect();

    if !missing.is_empty() || !stale.is_empty() {
        let mut msg = String::new();
        for (binary, package) in &missing {
            msg.push_str(&format!(
                "\n  {binary} ({package}) is release-only, but no `--test {binary}` \
                 appears in {WORKFLOW} and `{package}` is not in the `{RELEASE_JOB}` \
                 matrix — it runs in NO job"
            ));
        }
        for name in &stale {
            msg.push_str(&format!(
                "\n  allowlist entry `{name}` is stale — no release-only test binary \
                 by that name exists any more"
            ));
        }
        bail!(
            "release-only gates must be reached by the merge gate — named by `--test` or \
             in a package the `{RELEASE_JOB}` matrix runs wholesale — or allowlisted with \
             a reason:{msg}\n\nA release-only test skipped by every debug job and picked up \
             by no release job executes nowhere, while CI stays green (#747)."
        );
    }

    // Counted directly rather than by subtraction: an entry that is both
    // allowlisted and `--test`-named would make `len - named - exempt` underflow
    // a usize, which is a panic in debug and a nonsense total in release.
    let named = found.keys().filter(|b| listed.contains(*b)).count();
    let by_matrix = found
        .iter()
        .filter(|(binary, package)| !listed.contains(*binary) && wholesale.contains(*package))
        .count();
    println!(
        "{} {} release-only test binaries all reach a CI job — {named} named by `--test`, \
         {by_matrix} covered by the `{RELEASE_JOB}` matrix, {} allowlisted",
        "OK".green().bold(),
        found.len(),
        exempt.len(),
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ⚠ The regression this parser exists to avoid. Both spellings are live in
    /// `quality-gate.yml`, and a line-anchored pattern silently drops the inline
    /// one — which is how a covered gate got reported as missing by hand.
    #[test]
    fn listed_targets_reads_both_the_continuation_and_inline_spellings() {
        let yaml = "\
          - name: run\n\
            run: |\n\
              cargo test --release -p sim-soft \\\n\
                --test hertz_sphere_plane \\\n\
                --test contact_drop_rest\n\
          - name: heavy\n\
            run: cargo test --release -p sim-soft --test bonded_layer_indentation\n";
        let listed = listed_targets(yaml);
        assert!(
            listed.contains("hertz_sphere_plane"),
            "continuation form missed: {listed:?}"
        );
        assert!(
            listed.contains("contact_drop_rest"),
            "last continuation entry missed: {listed:?}"
        );
        assert!(
            listed.contains("bonded_layer_indentation"),
            "INLINE form missed — the exact false alarm this test exists for: {listed:?}"
        );
        assert_eq!(listed.len(), 3, "unexpected extras: {listed:?}");
    }

    /// ⚠ The second regression, and the one that made the first draft cry wolf:
    /// coverage also comes from a whole-package release run, and the `crates:`
    /// key is NOT unique to that job. A parser that unions every `crates:` line
    /// marks the cherry-picked package wholesale-covered and goes vacuous.
    #[test]
    fn wholesale_packages_come_only_from_the_release_job_matrix() {
        let yaml = "\
jobs:\n\
\x20 tests-debug:\n\
\x20   strategy:\n\
\x20     matrix:\n\
\x20       include:\n\
\x20         - crates: sim-soft sim-core\n\
\x20 tests-release:\n\
\x20   strategy:\n\
\x20     matrix:\n\
\x20       include:\n\
\x20         - crates: mesh-printability sim-opt\n\
\x20         - crates: cf-codesign\n\
\x20 cross-os:\n\
\x20   steps:\n\
\x20     - crates: mesh-io\n";
        let pkgs = wholesale_release_packages(yaml);
        assert!(pkgs.contains("mesh-printability"), "{pkgs:?}");
        assert!(
            pkgs.contains("cf-codesign"),
            "second matrix row missed: {pkgs:?}"
        );
        assert!(
            !pkgs.contains("sim-soft"),
            "a DEBUG shard's crates leaked in — this is what makes the check vacuous: {pkgs:?}"
        );
        assert!(!pkgs.contains("mesh-io"), "a later job leaked in: {pkgs:?}");
    }

    #[test]
    fn wholesale_packages_skips_an_expression_valued_matrix() {
        let yaml = "jobs:\n  tests-release:\n    crates: ${{ steps.compute.outputs.crates }}\n";
        assert!(wholesale_release_packages(yaml).is_empty());
    }

    /// ⚠ The false-NEGATIVE path, which is the dangerous direction: a `--test`
    /// in prose must not confer coverage a gate does not have. This workflow
    /// already carries three such mentions.
    #[test]
    fn a_test_flag_inside_a_comment_confers_no_coverage() {
        let yaml = "\
          # the `--test phantom_gate` selection was removed in #123\n\
          - run: cargo test --release -p sim-soft --test real_gate\n\
          jobs:\n\
          \x20 tests-release:\n\
          #         - crates: commented_out_pkg\n\
          \x20         - crates: real_pkg\n";
        let listed = listed_targets(yaml);
        assert!(listed.contains("real_gate"), "{listed:?}");
        assert!(
            !listed.contains("phantom_gate"),
            "a commented `--test` was read as coverage — the check would silently \
             stop protecting that gate: {listed:?}"
        );
        let pkgs = wholesale_release_packages(yaml);
        assert!(pkgs.contains("real_pkg"), "{pkgs:?}");
        assert!(
            !pkgs.contains("commented_out_pkg"),
            "a commented `crates:` line was read as coverage: {pkgs:?}"
        );
    }

    #[test]
    fn listed_targets_is_empty_on_a_workflow_naming_none() {
        assert!(listed_targets("jobs:\n  build:\n    run: cargo test\n").is_empty());
    }

    #[test]
    fn release_only_marker_matches_cfg_attr_and_not_a_plain_ignore() {
        let file: syn::File = syn::parse_str(
            "#[test]\n\
             #[cfg_attr(debug_assertions, ignore = \"release-only measurement\")]\n\
             fn release_only() {}\n\
             #[test]\n\
             #[ignore = \"needs $CF_L4_STL\"]\n\
             fn licence_gated() {}\n\
             #[test]\n\
             fn ordinary() {}\n",
        )
        .expect("fixture parses");
        let mut found = Vec::new();
        release_only_tests(&file.items, &mut found);
        assert_eq!(
            found,
            vec!["release_only".to_string()],
            "an unconditional #[ignore] is a DIFFERENT class (licence-gated) and \
             must not be swept in here"
        );
    }

    #[test]
    fn release_only_marker_descends_into_inline_modules() {
        let file: syn::File = syn::parse_str(
            "mod tests {\n\
               #[test]\n\
               #[cfg_attr(debug_assertions, ignore = \"release-only\")]\n\
               fn nested() {}\n\
             }\n",
        )
        .expect("fixture parses");
        let mut found = Vec::new();
        release_only_tests(&file.items, &mut found);
        assert_eq!(found, vec!["nested".to_string()]);
    }

    /// Unique scratch crate for one case — no external deps, mirroring
    /// `validators`' idiom. Returns a root holding a manifest and `tests/`.
    fn scratch(tag: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("xtask_release_gates_{tag}"));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(dir.join("tests")).expect("create scratch tests dir");
        std::fs::write(
            dir.join("Cargo.toml"),
            "[package]\nname = \"scratch-probe\"\n",
        )
        .expect("write scratch manifest");
        dir
    }

    /// ★ The refusal-to-guess guard. A `tests/*.rs` the scanner cannot parse
    /// holds an UNKNOWN number of release-only gates, not zero — swallowing it
    /// would silently un-protect that file while the total stayed plausible.
    /// Verified by hand when it was written; committed here because a
    /// hand-verified guard is an unverified one.
    #[test]
    fn an_unparseable_test_file_bails_rather_than_reading_as_no_gates() {
        let dir = scratch("unparseable");
        std::fs::write(dir.join("tests/broken.rs"), "this is not valid rust {{{\n")
            .expect("write broken test");

        let err = check_at(&dir).expect_err("must refuse to report success");
        let msg = err.to_string();
        assert!(
            msg.contains("could not read or parse"),
            "the unreadable guard did not fire: {msg}"
        );
        assert!(msg.contains("broken.rs"), "must name the file: {msg}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ★ The sibling guard: a scan finding nothing is a broken scan, not a clean
    /// workspace — the failure `licensed_gates` records from its own "0 gates
    /// across 0 crates" incident.
    #[test]
    fn an_empty_survey_bails_rather_than_reporting_all_clear() {
        let dir = scratch("empty");
        std::fs::write(dir.join("tests/plain.rs"), "#[test]\nfn ordinary() {}\n")
            .expect("write plain test");

        let err = check_at(&dir).expect_err("an empty survey must not read as success");
        assert!(
            err.to_string().contains("found NO release-only tests"),
            "wrong failure: {err}"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The invariant itself, as a unit test as well as a command — the command
    /// is what fires on a PR that does not touch xtask, and this is what keeps
    /// the two from drifting.
    #[test]
    fn every_release_only_gate_is_named_by_the_merge_gate() {
        // Rooted explicitly — never `set_current_dir`, which is process-global
        // and would race whichever sibling test reads a relative path.
        let workspace = Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("xtask has a parent workspace");
        check_at(workspace).expect("release-only gate coverage");
    }
}
