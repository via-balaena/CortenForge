//! Self-test: every workspace crate that HAS tests is named by a CI test job.
//!
//! [`crate::release_gates`] covers a gate CI can silently STOP running. This
//! covers the one before it — a crate whose tests CI never STARTED running.
//! Both are the same hole seen from different ends, and it has recurred three
//! times: crates left out of every shard's list, release-only tests in no
//! `--test` list (#747), and a whole crate scoped to `--lib` so its `tests/`
//! binaries were unreachable (#765, six tests red for 473 commits behind green
//! CI).
//!
//! ⚠ **A crate counts as having tests only via plain `#[test]`.** No
//! `#[tokio::test]`, `#[rstest]` or similar exists in this workspace today
//! (checked), so the count is complete — but a crate whose tests were all
//! written with such an attribute would count ZERO and be exempted in silence,
//! which is the exact failure mode this check exists to prevent. Widen
//! [`count_tests`] before introducing one.
//!
//! # What "reached" means
//!
//! A crate is REACHED when its package name appears in
//! `.github/workflows/quality-gate.yml` as a `-p <name>` argument, inside a job
//! matrix's crate list, or in an `affected-packages.sh "<list>"` argument. Those
//! are the three ways a package reaches a `cargo test` invocation in this
//! workflow.
//!
//! ⚠ Being named is necessary, not sufficient — this check does not verify that
//! the naming job actually runs the crate's tests, only that nothing is
//! unreachable by construction. A `--lib`-scoped invocation still *names* its
//! crate; that is exactly how #765's hole survived, and why the sibling
//! assertion in that step's comment carries the rest of the contract.
//!
//! ⚠ **`cargo xtask run-validators` does NOT count.** It executes each example
//! via `cargo run`, which never compiles a test target. An example crate with
//! `#[test]`s is therefore as unreachable as any other, and is reported as such
//! rather than waved through on the strength of the examples job existing.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use anyhow::{bail, Result};

use crate::licensed_gates::{owning_package, source_files};

/// Shown by `cargo xtask test-reachability --help`. The single statement of
/// what this asserts; CI's comment points here rather than restating it, because
/// a duplicated definition is what drifted the last three times.
pub const LONG_ABOUT: &str = "\
Assert that every workspace crate containing at least one #[test] is named by a
CI test job in .github/workflows/quality-gate.yml.

A crate is REACHED when its package name appears as a `-p <name>` argument, in a
job matrix's crate list, or in an `affected-packages.sh \"<list>\"` argument.

⚠ Necessary, not sufficient: this does not verify the naming job runs the
crate's tests. A `--lib`-scoped invocation still names its crate, which is how
six cf-cast tests stayed red for 473 commits behind green CI (#765).

⚠ `run-validators` does not count — it executes examples with `cargo run`, which
never builds a test target.

Needs no build, no licensed assets, and no network.";

const WORKFLOW: &str = ".github/workflows/quality-gate.yml";

/// Crates with tests that CI reaches by a mechanism this parser cannot see, each
/// with the reason. Deliberately explicit entries rather than a pattern, so an
/// exemption cannot be acquired by accident — and deliberately empty until one
/// is genuinely needed.
///
/// ⚠ A documented exclusion is still an exclusion (#765). An entry here must
/// name the mechanism that runs the tests, not merely assert someone looked.
const REACHED_ELSEWHERE: &[(&str, &str)] = &[];

/// Packages with at least one `#[test]`, and how many.
struct Survey {
    /// Package name → number of `#[test]` attributes found in its sources.
    with_tests: BTreeMap<String, usize>,
    /// Source files that could not be read or parsed. Reported, never silent:
    /// "I could not look" must not read the same as "there is nothing here".
    unreadable: Vec<std::path::PathBuf>,
}

fn survey(root: &Path) -> Survey {
    let mut with_tests: BTreeMap<String, usize> = BTreeMap::new();
    let mut unreadable = Vec::new();
    for path in source_files(root) {
        let Some((_, package)) = owning_package(&path) else {
            continue;
        };
        let Ok(text) = std::fs::read_to_string(&path) else {
            unreadable.push(path);
            continue;
        };
        let Ok(file) = syn::parse_file(&text) else {
            unreadable.push(path);
            continue;
        };
        let n = count_tests(&file.items);
        if n > 0 {
            *with_tests.entry(package).or_default() += n;
        }
    }
    Survey {
        with_tests,
        unreadable,
    }
}

/// Whether `package`, rooted at `crate_dir`, holds at least one plain
/// `#[test]`.
///
/// ★ Asked by [`crate::grade`] before it spends a pass-2 `cargo test --release`
/// on a crate whose COVERAGE is N/A. Measured 2026-08-27: 240 of 301 workspace
/// crates take that path and only **16** have any tests, so running the suite
/// for all of them would buy 224 release builds that cannot find anything —
/// 223 of them `examples/` crates carrying the Bevy tree.
///
/// ⚠ **FAILS OPEN, and deliberately the opposite way to [`survey`].** A file
/// that cannot be read or parsed counts as tests-present, so the crate is
/// tested rather than skipped. `survey` reports unreadable files instead,
/// because its question is "did I look everywhere"; here the two errors are not
/// symmetric — a needless test run costs minutes, a silently skipped one is the
/// exact defect this call exists to close.
///
/// ⚠ Plain `#[test]` only, with the same blind spot the module doc records: a
/// crate whose tests were all `#[tokio::test]` would read as having none and be
/// skipped in silence. None exists in this workspace today (checked); widen
/// [`count_tests`] before introducing one.
pub(crate) fn crate_has_tests(crate_dir: &Path, package: &str) -> bool {
    for path in source_files(crate_dir) {
        // Scoped by OWNING PACKAGE, not by the walk: `source_files` recurses,
        // and a crate directory that contains a nested crate would otherwise
        // borrow its tests and claim a suite it does not have.
        match owning_package(&path) {
            Some((_, owner)) if owner == package => {}
            _ => continue,
        }
        let Ok(text) = std::fs::read_to_string(&path) else {
            return true;
        };
        let Ok(file) = syn::parse_file(&text) else {
            return true;
        };
        if count_tests(&file.items) > 0 {
            return true;
        }
    }
    false
}

/// Count `#[test]` functions, descending into modules so a crate that hides its
/// tests behind `mod tests` is not read as having none.
fn count_tests(items: &[syn::Item]) -> usize {
    let mut n = 0;
    for item in items {
        match item {
            syn::Item::Fn(f) => {
                if f.attrs.iter().any(|a| a.path().is_ident("test")) {
                    n += 1;
                }
            }
            syn::Item::Mod(m) => {
                if let Some((_, inner)) = &m.content {
                    n += count_tests(inner);
                }
            }
            _ => {}
        }
    }
    n
}

/// Package names the workflow mentions in a way that can reach `cargo test`.
pub(crate) fn crates_named_in_workflow(yaml: &str) -> BTreeSet<String> {
    let mut named = BTreeSet::new();
    let lines: Vec<&str> = yaml.lines().collect();
    for (i, raw) in lines.iter().enumerate() {
        let line = raw.trim_end();
        let trimmed = line.trim_start();
        // A commented-out job is not a running job; reading one as coverage is
        // the false clean bill this check exists to prevent.
        if trimmed.starts_with('#') {
            continue;
        }
        // `-p <name>` anywhere in a run command.
        let mut rest = trimmed;
        while let Some(idx) = rest.find("-p ") {
            rest = &rest[idx + 3..];
            let name: String = rest
                .chars()
                .take_while(|c| c.is_ascii_alphanumeric() || *c == '-' || *c == '_')
                .collect();
            if !name.is_empty() {
                named.insert(name);
            }
        }
        // `affected-packages.sh "<list>"` arguments.
        if let Some((_, tail)) = trimmed.split_once("affected-packages.sh \"") {
            if let Some((list, _)) = tail.split_once('"') {
                extend_names(&mut named, list);
            }
        }
        // `crates: a b c` — ⚠ a YAML plain scalar, which CONTINUES onto more
        // indented lines. Reading only this line is how the first draft of this
        // check reported 43 orphans that were in fact listed; the continuation
        // lines carry most of the workspace.
        let Some((head, value)) = line.split_once("crates:") else {
            continue;
        };
        if value.contains("${{") {
            continue; // an expression, not a bare name list
        }
        extend_names(&mut named, value);
        let key_indent = head.len() - head.trim_start().len();
        for cont in lines.iter().skip(i + 1) {
            let body = cont.trim();
            let indent = cont.len() - cont.trim_start().len();
            let is_continuation = indent > key_indent
                && !body.is_empty()
                && !body.starts_with('#')
                && body.split_whitespace().all(|t| {
                    t.chars()
                        .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
                });
            if !is_continuation {
                break;
            }
            extend_names(&mut named, body);
        }
    }
    named
}

/// Add whitespace-separated crate-name tokens, ignoring anything that is not a
/// bare package name.
fn extend_names(named: &mut BTreeSet<String>, list: &str) {
    for tok in list.split_whitespace() {
        if !tok.is_empty()
            && tok
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
        {
            named.insert(tok.to_string());
        }
    }
}

/// Assert every crate with tests is named by a CI test job.
///
/// # Errors
///
/// Returns an error if the workflow cannot be read, if either enumeration comes
/// back empty (which would mean the parser broke, not that the tree is clean),
/// if any source file could not be parsed, or if a crate with tests is
/// unreachable.
pub fn check() -> Result<()> {
    check_at(Path::new("."))
}

/// [`check`], rooted at an explicit workspace directory so the unit test can
/// point at the workspace without `set_current_dir` — a global mutation that
/// would race whichever sibling test happens to read a relative path.
pub(crate) fn check_at(root: &Path) -> Result<()> {
    let Survey {
        with_tests,
        unreadable,
    } = survey(root);

    if !unreadable.is_empty() {
        bail!(
            "could not parse {} source file(s), so this check cannot claim the tree is \
             clean — a scan that silently skips files reports 0 orphans no matter what \
             is there. First few: {:?}",
            unreadable.len(),
            unreadable.iter().take(5).collect::<Vec<_>>()
        );
    }
    if with_tests.is_empty() {
        bail!("found NO crate with a #[test] anywhere — the survey is broken");
    }

    let workflow = root.join(WORKFLOW);
    let yaml = std::fs::read_to_string(&workflow)
        .map_err(|e| anyhow::anyhow!("read {}: {e}", workflow.display()))?;
    let named = crates_named_in_workflow(&yaml);
    if named.is_empty() {
        bail!("parsed NO crate names out of {WORKFLOW} — the parser is broken");
    }

    let exempt: BTreeMap<&str, &str> = REACHED_ELSEWHERE.iter().copied().collect();
    let mut orphans: Vec<(&String, usize)> = with_tests
        .iter()
        .filter(|(pkg, _)| !named.contains(pkg.as_str()) && !exempt.contains_key(pkg.as_str()))
        .map(|(pkg, n)| (pkg, *n))
        .collect();
    orphans.sort_by_key(|(_, n)| std::cmp::Reverse(*n));

    // A stale exemption is its own defect: it says "CI handles this" about a
    // crate CI may since have started naming, and nothing else would notice.
    let stale: Vec<&str> = exempt
        .keys()
        .filter(|pkg| named.contains(**pkg) || !with_tests.contains_key(**pkg))
        .copied()
        .collect();
    if !stale.is_empty() {
        bail!(
            "REACHED_ELSEWHERE has rotted — {stale:?} either no longer has tests or is \
             now named in {WORKFLOW}. Remove the entries; an exemption nobody rechecks \
             is how the exclusion outlives its reason."
        );
    }

    if orphans.is_empty() {
        println!(
            "✓ all {} crate(s) with tests are named by a CI test job",
            with_tests.len()
        );
        return Ok(());
    }

    let total: usize = orphans.iter().map(|(_, n)| n).sum();
    let detail = orphans
        .iter()
        .map(|(pkg, n)| format!("  {pkg} ({n} tests)"))
        .collect::<Vec<_>>()
        .join("\n");
    bail!(
        "{} crate(s) with tests are named by NO CI test job — {total} tests run \
         nowhere:\n{detail}\n\nAdd each to a job's crate list in {WORKFLOW}, or add an \
         entry to REACHED_ELSEWHERE naming the mechanism that runs its tests. \
         ⚠ `run-validators` is not such a mechanism: it runs `cargo run`, which never \
         builds a test target.",
        orphans.len()
    );
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn counts_tests_inside_nested_modules() {
        let file = syn::parse_file(
            "#[test] fn a() {} mod outer { mod inner { #[test] fn b() {} } fn c() {} }",
        )
        .unwrap();
        assert_eq!(
            count_tests(&file.items),
            2,
            "a crate hiding tests in `mod tests` must not read as having none"
        );
    }

    #[test]
    fn a_file_with_no_tests_counts_zero() {
        let file = syn::parse_file("fn a() {} mod m { fn b() {} }").unwrap();
        assert_eq!(count_tests(&file.items), 0);
    }

    #[test]
    fn reads_package_names_from_all_three_shapes() {
        let yaml = "
      - run: cargo test -p sim-core --release
        crates: cf-viewer sim-bevy-soft
      - run: |
          PKGS=$(bash .github/scripts/affected-packages.sh \"mesh-repair mesh-shell\")
";
        let named = crates_named_in_workflow(yaml);
        for want in [
            "sim-core",
            "cf-viewer",
            "sim-bevy-soft",
            "mesh-repair",
            "mesh-shell",
        ] {
            assert!(named.contains(want), "{want} not parsed out of {named:?}");
        }
    }

    /// ★ Regression for the bug this parser shipped with in its first draft: a
    /// `crates:` value is a YAML PLAIN SCALAR and continues onto more-indented
    /// lines. Reading only the first line reported 43 orphans when there were
    /// 10 — the continuation lines carry most of the workspace, so the check
    /// would have been loudly, confidently wrong.
    #[test]
    fn a_crates_list_continuing_onto_indented_lines_is_read_whole() {
        let yaml = "\
          - id: 3
            crates: sim-soft sim-core sim-mjcf
              sim-therm-env cf-design cf-fsu-geometry
              pbit-analyze pbit-fw-core
          - id: 4
            crates: mesh-io
";
        let named = crates_named_in_workflow(yaml);
        for want in [
            "sim-soft",
            "sim-therm-env",
            "cf-design",
            "pbit-fw-core",
            "mesh-io",
        ] {
            assert!(named.contains(want), "{want} missing from {named:?}");
        }
        assert!(
            !named.contains("id"),
            "the next list item must end the scalar, not be swallowed: {named:?}"
        );
    }

    /// ★ The parser must not credit a COMMENTED-OUT crate. A commented job is
    /// not a running job, and reading one as coverage is exactly the false
    /// clean bill this check exists to prevent.
    #[test]
    fn a_commented_out_crate_is_not_reached() {
        let named = crates_named_in_workflow("# - run: cargo test -p ghost-crate");
        assert!(
            !named.contains("ghost-crate"),
            "a commented-out line must not count as reaching a crate"
        );
    }

    /// ★ The real workspace passes. Also the negative control for the whole
    /// check: if `crates_named_in_workflow` silently returned everything, or
    /// `survey` silently returned nothing, this would still pass — so the two
    /// emptiness bails above are what make a broken scan loud rather than green.
    #[test]
    fn the_workspace_has_no_orphaned_test_crates() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap();
        check_at(root).expect("every crate with tests should be named by a CI job");
    }

    /// A throwaway crate tree: `(relative path, contents)`, plus a manifest
    /// naming the package.
    fn crate_fixture(tag: &str, package: &str, files: &[(&str, &str)]) -> std::path::PathBuf {
        let root = std::env::temp_dir().join(format!(
            "cf-has-tests-{tag}-{}-{:?}",
            std::process::id(),
            std::thread::current().id()
        ));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).expect("mkdir root");
        std::fs::write(
            root.join("Cargo.toml"),
            format!("[package]\nname = \"{package}\"\n"),
        )
        .expect("write manifest");
        for (name, body) in files {
            let path = root.join(name);
            std::fs::create_dir_all(path.parent().expect("parent")).expect("mkdir");
            std::fs::write(&path, body).expect("write");
        }
        root
    }

    /// The decision `grade` spends a release test run on.
    ///
    /// ★ BOTH FACES. A prober that always answers `true` costs 224 needless
    /// release builds; one that always answers `false` silently reinstates the
    /// defect this whole change closes. Neither assertion alone can tell the
    /// difference.
    #[test]
    fn crate_has_tests_finds_a_nested_test_module_and_says_no_when_there_is_none() {
        let with = crate_fixture(
            "with",
            "widget",
            &[(
                "src/lib.rs",
                "pub fn f() {}\n#[cfg(test)]\nmod tests {\n  #[test]\n  fn t() {}\n}\n",
            )],
        );
        assert!(
            crate_has_tests(&with, "widget"),
            "a #[test] inside `mod tests` is the normal shape and must be found"
        );

        let without = crate_fixture("without", "widget", &[("src/lib.rs", "pub fn f() {}\n")]);
        assert!(
            !crate_has_tests(&without, "widget"),
            "a crate with no #[test] must answer false, or the prober is `true` \
             and buys a release build for all 223 example crates"
        );
    }

    /// Unparseable input must answer YES.
    ///
    /// ⚠ The opposite default to [`survey`], and deliberately so: there, an
    /// unreadable file is REPORTED because the question is "did I look
    /// everywhere". Here the errors are asymmetric — answering yes wastes a
    /// test run, answering no skips a suite in silence, which is the defect.
    #[test]
    fn crate_has_tests_fails_open_on_a_file_it_cannot_parse() {
        let broken = crate_fixture(
            "broken",
            "widget",
            &[("src/lib.rs", "this is not rust {{{ \n")],
        );
        assert!(
            crate_has_tests(&broken, "widget"),
            "a file that will not parse must be treated as possibly holding tests"
        );
    }

    /// A nested crate's tests belong to the nested crate.
    ///
    /// ⚠ `source_files` recurses, so without the owning-package filter a crate
    /// directory containing another crate would borrow its tests and claim a
    /// suite it does not have — then run `cargo test -p <outer>`, which would
    /// not execute them.
    #[test]
    fn crate_has_tests_does_not_borrow_a_nested_packages_tests() {
        let root = crate_fixture("nested", "outer", &[("src/lib.rs", "pub fn f() {}\n")]);
        let inner = root.join("inner");
        std::fs::create_dir_all(inner.join("src")).expect("mkdir inner");
        std::fs::write(inner.join("Cargo.toml"), "[package]\nname = \"inner\"\n")
            .expect("write inner manifest");
        std::fs::write(
            inner.join("src/lib.rs"),
            "#[cfg(test)]\nmod tests {\n  #[test]\n  fn t() {}\n}\n",
        )
        .expect("write inner source");

        assert!(
            !crate_has_tests(&root, "outer"),
            "outer has no tests of its own; inner's must not count for it"
        );
        // Positive control: the walk DOES reach the nested file, so the
        // assertion above is about ownership and not about an empty walk.
        assert!(
            crate_has_tests(&root, "inner"),
            "the nested file is reachable from the same walk — otherwise the \
             assertion above would pass on a walk that found nothing"
        );
    }
}
