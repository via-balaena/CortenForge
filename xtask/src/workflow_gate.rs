//! Self-test: every gating CI job is wired into the `quality-gate` aggregator.
//!
//! The `main` ruleset requires exactly one status check — the `quality-gate`
//! job — which `needs:` every other gating job. That indirection is what lets
//! us rename/shard individual jobs without touching branch protection, but it
//! has a failure mode: a NEW job that runs a real check yet is left out of
//! `quality-gate.needs` would execute UN-GATED — its red never blocks a merge,
//! because the required aggregator never waited for it.
//!
//! This module is compiled only under `cfg(test)` (see `main.rs`) and hosts a
//! test that parses the workflow and asserts every job except an explicit
//! advisory allowlist is listed in `quality-gate`'s `needs`. So "added a job,
//! forgot to gate it" becomes a hard test failure — the CI config validates its
//! own completeness. It also catches a `needs` entry naming a job that no longer
//! exists (a rename/typo), and a stale allowlist entry.

/// Jobs intentionally NOT gated by `quality-gate`, each for a concrete reason:
/// - `affected` — infra that computes the PR-scope; there is nothing to gate.
/// - `quality-gate` — the aggregator itself; it cannot `need` itself.
/// - `semver` / `sbom` — advisory (breaking-change detection / SBOM artifact),
///   deliberately non-blocking.
///
/// Adding a job forces an explicit choice: wire it into `needs`, or justify an
/// entry here. The test asserts every name here still exists, so the allowlist
/// can't rot silently.
const UNGATED_JOBS: &[&str] = &["affected", "quality-gate", "semver", "sbom"];

/// Extract the top-level job names from a GitHub Actions workflow.
///
/// A job key is a line indented exactly two spaces under `jobs:`, bare (`  x:`),
/// naming only `[A-Za-z0-9_-]`. Deeper-indented keys, comments, and mapping
/// values (`  runs-on: ubuntu`, which does not end in `:`) are skipped.
fn parse_jobs(yaml: &str) -> Vec<String> {
    let mut in_jobs = false;
    let mut jobs = Vec::new();
    for raw in yaml.lines() {
        let line = raw.trim_end();
        if line == "jobs:" {
            in_jobs = true;
            continue;
        }
        if !in_jobs {
            continue;
        }
        if let Some(rest) = line.strip_prefix("  ") {
            // A deeper key (leading space) or comment is not a job.
            if rest.starts_with(' ') || rest.starts_with('#') {
                continue;
            }
            if let Some(name) = rest.strip_suffix(':') {
                if !name.is_empty()
                    && name
                        .chars()
                        .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
                {
                    jobs.push(name.to_string());
                }
            }
        }
    }
    jobs
}

/// Parse the inline `needs: [a, b, c]` list of a named job. Returns empty if the
/// job or an inline `needs` array is absent (the caller asserts non-empty).
fn parse_needs(yaml: &str, job: &str) -> Vec<String> {
    let header = format!("  {job}:");
    let mut in_job = false;
    for raw in yaml.lines() {
        let line = raw.trim_end();
        if line == header {
            in_job = true;
            continue;
        }
        if !in_job {
            continue;
        }
        // A new top-level job key ends this job's block.
        if let Some(rest) = line.strip_prefix("  ") {
            if !rest.starts_with(' ')
                && !rest.starts_with('#')
                && rest.strip_suffix(':').is_some_and(|n| {
                    !n.is_empty()
                        && n.chars()
                            .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
                })
            {
                break;
            }
        }
        if let Some(list) = line.trim_start().strip_prefix("needs:") {
            let inner = list.trim().trim_start_matches('[').trim_end_matches(']');
            return inner
                .split(',')
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty())
                .collect();
        }
    }
    Vec::new()
}

/// The workflow, embedded at compile time so the test needs no runtime path and
/// a moved/renamed workflow breaks the build (a loud signal, not a silent skip).
const WORKFLOW: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../.github/workflows/quality-gate.yml"
));

#[test]
fn every_gating_job_is_in_quality_gate_needs() {
    let jobs = parse_jobs(WORKFLOW);
    assert!(
        jobs.len() >= 10,
        "parser found only {} job(s) — the workflow format likely changed: {jobs:?}",
        jobs.len()
    );

    let needs = parse_needs(WORKFLOW, "quality-gate");
    assert!(
        !needs.is_empty(),
        "could not parse `quality-gate` needs (format changed?)"
    );

    // 1. Every job except the advisory allowlist must be gated — otherwise it
    //    runs where a red result never blocks a merge.
    for job in &jobs {
        if UNGATED_JOBS.contains(&job.as_str()) {
            continue;
        }
        assert!(
            needs.contains(job),
            "job `{job}` is not in `quality-gate` needs and is not on the advisory \
             allowlist — it would run UN-GATED (its failure could not block a merge). \
             Add it to `quality-gate.needs`, or to UNGATED_JOBS if it is intentionally \
             advisory."
        );
    }

    // 2. Every `needs` entry must name a real job (catch a rename/typo that would
    //    make the aggregator wait on — or skip — a phantom).
    for n in &needs {
        assert!(
            jobs.contains(n),
            "`quality-gate` needs `{n}`, which is not a job in this workflow \
             (renamed or misspelled?)."
        );
    }

    // 3. The allowlist itself must not rot: every exempt name must still exist.
    for exempt in UNGATED_JOBS {
        assert!(
            jobs.iter().any(|j| j == exempt),
            "UNGATED_JOBS names `{exempt}`, which is no longer a job — prune the allowlist."
        );
    }
}

/// A GATED job must not be conditional on the event type.
///
/// ⚠⚠ This nearly broke the merge queue for the whole repository. The
/// `doc-theft` job first carried `if: github.event_name == 'pull_request'`.
/// This workflow also runs on `merge_group` and `push`; a job whose `if` is
/// false reports **`skipped`**, and `quality-gate` fails anything that is not
/// exactly `success`. Every merge-queue run would have failed the required
/// check, so nothing could ever have landed on main.
///
/// `semver` carries the same condition and is harmless ONLY because it is
/// advisory — it is on [`UNGATED_JOBS`] and nothing waits on it. That made the
/// pattern look safe by example, which is exactly why this needs a test rather
/// than a habit.
///
/// The fix for such a job is not to drop the condition but to move it: let the
/// job always run, and have the STEP decide what it can do for that event.
#[test]
fn no_gated_job_is_conditional_on_the_event_type() {
    let needs = parse_needs(WORKFLOW, "quality-gate");
    assert!(!needs.is_empty(), "could not parse `quality-gate` needs");

    let mut current: Option<String> = None;
    let mut in_steps = false;
    let mut offenders: Vec<String> = Vec::new();

    for raw in WORKFLOW.lines() {
        // A top-level job key: exactly two spaces of indent, then `name:`.
        if let Some(rest) = raw.strip_prefix("  ") {
            if !rest.starts_with(' ') && !rest.starts_with('#') {
                if let Some(name) = rest.strip_suffix(':') {
                    if !name.contains(' ') {
                        current = Some(name.to_string());
                        in_steps = false;
                        continue;
                    }
                }
            }
        }
        if raw.trim_start().starts_with("steps:") {
            in_steps = true;
        }
        // Only the job-level `if:` matters; a step-level one is the correct fix.
        if !in_steps && raw.trim_start().starts_with("if:") && raw.contains("github.event_name") {
            if let Some(job) = current.as_deref() {
                if needs.iter().any(|n| n == job) {
                    offenders.push(job.to_string());
                }
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "gated job(s) {offenders:?} are conditional on `github.event_name`. A \
         skipped job reports `skipped`, and `quality-gate` fails anything that \
         is not `success` — so every merge_group and push run of the required \
         check would fail and nothing could merge. Move the condition into the \
         STEP and let the job always run."
    );
}

#[test]
fn parse_jobs_extracts_top_level_keys_only() {
    let yaml = "\
name: CI
on:
  push:
    branches: [main]
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - run: cargo build
  # a comment
  test:
    needs: [build]
    steps:
      - run: cargo test
";
    assert_eq!(parse_jobs(yaml), vec!["build", "test"]);
}

#[test]
fn parse_needs_reads_inline_array() {
    let yaml = "\
jobs:
  build:
    needs: [affected]
  gate:
    needs: [build, test, lint]
    steps:
      - run: true
  after:
    runs-on: ubuntu-latest
";
    assert_eq!(parse_needs(yaml, "gate"), vec!["build", "test", "lint"]);
    // Stops at the job block boundary — `after` is not conflated in.
    assert_eq!(parse_needs(yaml, "build"), vec!["affected"]);
    assert!(parse_needs(yaml, "missing").is_empty());
}

// ============================================================================
// The shared Linux dependency list
// ============================================================================
// Eleven jobs across three workflows install the same apt packages through
// `install-linux-deps`, and the list is written out longhand at every one.
// `release.yml` states the hazard on its own copy: a divergent list on the
// BLOCKING release leg "would fail only at tag time, invisible on every PR".
//
// It has already bitten twice in the other direction, on the same 2026-08-18
// weekly run: the bench job had no dependency step at all, and clippy-drift's
// list was missing a package. Both died in a dependency's build script before
// running anything, and the reds were read as a bench regression and as
// toolchain drift rather than as what they were.
//
// So the list gets the same treatment as `quality-gate.needs`: enforced, not
// asserted in a comment. Editing ten of the eleven copies is now a test failure.

/// The packages every Linux job installs.
const LINUX_DEPS: &[&str] = &[
    "libwayland-dev",
    "libxkbcommon-dev",
    "libasound2-dev",
    "libudev-dev",
];

/// Packages a job may add on top of [`LINUX_DEPS`] for a reason local to it.
/// `mesa-vulkan-drivers` provides lavapipe, a CPU Vulkan device — the jobs that
/// execute `sim-gpu`'s suite need an adapter or those tests return `ok` having
/// measured nothing.
const LINUX_DEPS_OPTIONAL: &[&str] = &["mesa-vulkan-drivers"];

/// Every workflow file, read from disk at test time.
///
/// Enumerated rather than listed by name: a NEW workflow that installs Linux
/// deps has to be covered the day it lands, and a hard-coded list would simply
/// not see it — the same hole [`every_gating_job_is_in_quality_gate_needs`]
/// closes for a new job. The count assert below is what keeps a wrong path from
/// turning this into a gate that reads nothing.
fn workflow_files() -> Vec<(String, String)> {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../.github/workflows");
    let Ok(entries) = std::fs::read_dir(dir) else {
        // No panic: an unreadable directory yields no files, and the count
        // assert below reports that far better than a read error would.
        return Vec::new();
    };
    let mut files: Vec<(String, String)> = entries
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| {
            path.extension()
                .is_some_and(|ext| ext == "yml" || ext == "yaml")
        })
        .map(|path| {
            let name = path
                .file_name()
                .unwrap_or_default()
                .to_string_lossy()
                .into_owned();
            (name, std::fs::read_to_string(&path).unwrap_or_default())
        })
        .collect();
    files.sort();
    files
}

/// Count the steps that invoke the Linux deps action.
///
/// The mirror oracle for [`parse_package_lists`]: the action takes exactly one
/// `packages:` input per use, so the two counts must agree. Without this the
/// parser's blind spots are silent — `with: {packages: …}` on one line is valid
/// YAML that GitHub accepts and really does install what it names, but the
/// line-oriented reader below cannot see it, and a floor on the site count
/// cannot tell "ten sites" from "eleven sites, one of them unread".
fn count_action_uses(yaml: &str) -> usize {
    yaml.lines()
        .filter(|line| {
            let trimmed = line.trim_start();
            trimmed.starts_with("uses:") && trimmed.contains("install-linux-deps")
        })
        .count()
}

/// Drop a trailing YAML comment. A package name cannot contain `" #"`, so the
/// first occurrence ends the value — and this file comments nearly everything,
/// so an annotated package list is an edit to expect, not a malformed one.
fn strip_comment(s: &str) -> &str {
    match s.find(" #") {
        Some(i) => &s[..i],
        None => s,
    }
}

/// Every `packages:` input in a workflow, folded continuation lines included.
///
/// A value may run onto following lines, which YAML folds into one scalar:
///
/// ```text
///   packages: libwayland-dev libxkbcommon-dev libasound2-dev libudev-dev
///     mesa-vulkan-drivers
/// ```
///
/// A continuation is more-indented than the `packages:` key and carries no `:`
/// of its own, which is what separates it from the next mapping key.
fn parse_package_lists(yaml: &str) -> Vec<Vec<String>> {
    let mut lists = Vec::new();
    let mut lines = yaml.lines().peekable();
    while let Some(raw) = lines.next() {
        let trimmed = raw.trim_start();
        let Some(tail) = trimmed.strip_prefix("packages:") else {
            continue;
        };
        let indent = raw.len() - trimmed.len();
        let mut pkgs: Vec<String> = strip_comment(tail)
            .split_whitespace()
            .map(str::to_owned)
            .collect();
        while let Some(next) = lines.peek() {
            let t = next.trim_start();
            if t.is_empty()
                || t.starts_with('#')
                || t.contains(':')
                || next.len() - t.len() <= indent
            {
                break;
            }
            pkgs.extend(strip_comment(t).split_whitespace().map(str::to_owned));
            lines.next();
        }
        lists.push(pkgs);
    }
    lists
}

#[test]
fn every_linux_job_installs_the_same_dep_list() {
    let files = workflow_files();
    assert!(
        files.len() >= 4,
        "found only {} workflow file(s) — the path is wrong, and a gate that reads \
         no workflows passes while checking nothing",
        files.len()
    );
    let sites: Vec<(&str, Vec<String>)> = files
        .iter()
        .flat_map(|(name, yaml)| {
            parse_package_lists(yaml)
                .into_iter()
                .map(move |pkgs| (name.as_str(), pkgs))
        })
        .collect();

    assert!(
        sites.len() >= 10,
        "parser found only {} `packages:` input(s) across {} workflow(s) — the \
         format likely changed, and a gate that parses nothing passes silently",
        sites.len(),
        files.len()
    );

    let uses: usize = files.iter().map(|(_, yaml)| count_action_uses(yaml)).sum();
    assert_eq!(
        sites.len(),
        uses,
        "{uses} step(s) invoke install-linux-deps but {} `packages:` input(s) were \
         read, so a call site is installing packages this gate never checked. Write \
         the input as `packages:` on its own line under `with:` — an inline \
         `with: {{packages: …}}` is valid YAML that this reader cannot see.",
        sites.len()
    );

    // The optional list must not rot: an entry nothing installs any more would
    // go on silently permitting a package. Same rule as `UNGATED_JOBS` above.
    for extra in LINUX_DEPS_OPTIONAL {
        assert!(
            sites
                .iter()
                .any(|(_, pkgs)| pkgs.iter().any(|p| p == extra)),
            "LINUX_DEPS_OPTIONAL allows `{extra}`, which no job installs any more — \
             prune it, or the exemption outlives the reason for it."
        );
    }

    for (workflow, pkgs) in &sites {
        let base: Vec<&str> = pkgs
            .iter()
            .map(String::as_str)
            .filter(|p| !LINUX_DEPS_OPTIONAL.contains(p))
            .collect();
        assert_eq!(
            base, LINUX_DEPS,
            "a Linux dep list in `{workflow}` has drifted from the shared list. \
             Every `install-linux-deps` call site installs the same packages, and \
             the copies are longhand — so editing one means editing all of them \
             (`grep -rn 'packages: libwayland' .github/`). A divergence on the \
             release leg would surface only at tag time. Found: {pkgs:?}"
        );
    }
}

#[test]
fn parse_package_lists_folds_continuations_and_stops_at_the_next_key() {
    let yaml = "\
      - name: Install system dependencies
        uses: ./.github/actions/install-linux-deps
        with:
          packages: libwayland-dev libudev-dev
            mesa-vulkan-drivers

      - name: Install Rust
        uses: dtolnay/rust-toolchain@stable
        with:
          packages: libwayland-dev
";
    assert_eq!(
        parse_package_lists(yaml),
        vec![
            vec!["libwayland-dev", "libudev-dev", "mesa-vulkan-drivers"],
            vec!["libwayland-dev"],
        ]
    );
    // A same-indent sibling key is not swallowed as a continuation.
    assert_eq!(
        parse_package_lists("          packages: a\n          other: b\n"),
        vec![vec!["a"]]
    );
    // A trailing comment is not part of the value, on the key line or a
    // continuation — YAML ends a plain scalar at ` #`, and so must this.
    assert_eq!(
        parse_package_lists(
            "          packages: libudev-dev  # why\n            libasound2-dev # and\n"
        ),
        vec![vec!["libudev-dev", "libasound2-dev"]]
    );
}

/// The release-notes template, which carries the same install instructions.
///
/// `include_str!` rather than a lookup in [`workflow_files`]: a renamed
/// release.yml should break the build loudly, not quietly drop a source this
/// gate believes it is checking.
const RELEASE: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../.github/workflows/release.yml"
));

/// Every published page. Enumerated for the same reason as [`workflow_files`]:
/// a page added later that carries install instructions must be covered without
/// anyone remembering to add it here.
fn site_pages() -> Vec<(String, String)> {
    fn walk(root: &std::path::Path, dir: &std::path::Path, out: &mut Vec<(String, String)>) {
        let Ok(entries) = std::fs::read_dir(dir) else {
            return;
        };
        for path in entries.filter_map(Result::ok).map(|e| e.path()) {
            if path.is_dir() {
                walk(root, &path, out);
            } else if path.extension().is_some_and(|ext| ext == "html") {
                let body = std::fs::read_to_string(&path).unwrap_or_default();
                // Named against the ROOT, not the directory being walked: an
                // assert message is read by a person, and every nested page
                // would otherwise report as the same bare file name.
                let shown = path.strip_prefix(root).map_or_else(
                    |_| path.display().to_string(),
                    |rel| rel.display().to_string(),
                );
                out.push((shown, body));
            }
        }
    }
    let root = std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../site"));
    let mut pages = Vec::new();
    walk(root, root, &mut pages);
    pages.sort();
    pages
}

/// Every `lib…-dev` token in a document, wherever it appears.
///
/// Deliberately not scoped to the `apt-get` block: the defect below stated the
/// package in a command AND justified it in the sentence above, and only the
/// command would have been caught by a narrower reader.
fn dev_packages_named_in(text: &str) -> Vec<String> {
    let mut found: Vec<String> = text
        .split(|c: char| !(c.is_ascii_alphanumeric() || c == '-' || c == '.'))
        .filter(|t| t.starts_with("lib") && t.ends_with("-dev"))
        .map(str::to_owned)
        .collect();
    found.sort_unstable();
    found.dedup();
    found
}

/// The install instructions must not name a library CI does not install.
///
/// ⚠ This is not hypothetical tidiness. The install page told readers to install
/// `libfontconfig1-dev` and gave "the build fails inside `yeslogic-fontconfig-sys`"
/// as the reason, for months after the Slint→Bevy port removed that crate from the
/// lockfile entirely. The page is published on merge, so the false instruction was
/// user-facing, and nothing connected it to the CI lists it was meant to mirror.
#[test]
fn the_install_instructions_name_no_library_ci_does_not_install() {
    let pages = site_pages();
    assert!(
        pages.len() >= 2,
        "found only {} page(s) under site/ — the path is wrong, and release.yml \
         alone would satisfy the count below while no page was ever read",
        pages.len()
    );
    let mut sources: Vec<(String, String)> = pages
        .into_iter()
        .map(|(name, body)| (format!("site/{name}"), body))
        .collect();
    sources.push(("release.yml release notes".to_owned(), RELEASE.to_owned()));

    let mut named_anywhere = 0;
    for (source, text) in &sources {
        for pkg in dev_packages_named_in(text) {
            named_anywhere += 1;
            assert!(
                LINUX_DEPS.contains(&pkg.as_str()) || LINUX_DEPS_OPTIONAL.contains(&pkg.as_str()),
                "{source} names `{pkg}`, which no CI job installs. \
                 Either the workspace genuinely needs it (add it to LINUX_DEPS and to \
                 every `install-linux-deps` call site, so CI proves it is needed), or \
                 the instructions have outlived the dependency and should drop it."
            );
        }
    }
    assert!(
        named_anywhere > 0,
        "no `lib…-dev` named in any of the {} source(s) — the install instructions \
         moved or changed shape, and this gate would pass while checking nothing",
        sources.len()
    );
}
